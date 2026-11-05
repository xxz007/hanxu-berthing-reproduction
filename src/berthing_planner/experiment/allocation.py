"""Allocate fixed A2 scans to two UUVs and plan a timed surface-support USV.

This run reuses the full original coverage target and active task footprints.
Only point-position geometry, task accounting and a declared mission clock are
evaluated. Deployment/recovery and 3D vehicle feasibility are not simulated.
"""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]

import numpy as np
from scipy.spatial import cKDTree

from berthing_planner.cooperative_allocation import allocate_scan_tasks, plan_scan_tasks
from berthing_planner.mission_timing import (
    pairwise_distance_extrema, plan_departure_delays, position_at, time_parameterize,
)
from berthing_planner.occupancy import OccupancyQuadTree
from berthing_planner.search import astar, shortcut
from berthing_planner.experiment.chart_geometry import Geos


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def complete_holds(timeline, end):
    """Make implicit deployed-start and post-completion holds explicit."""
    pieces = list(timeline['segments'])
    if timeline['start_time_s'] > 0:
        p = timeline['initial_position']
        pieces.insert(0, {'t0_s': 0.0, 't1_s': timeline['start_time_s'],
                         'p0': p, 'p1': p, 'scanning': False, 'kind': 'hold'})
    if timeline['finish_time_s'] < end:
        p = timeline['final_position']
        pieces.append({'t0_s': timeline['finish_time_s'], 't1_s': end,
                       'p0': p, 'p1': p, 'scanning': False, 'kind': 'hold'})
    return {**timeline, 'segments': pieces, 'clock_end_s': end,
            'movement_start_time_s': timeline['start_time_s'],
            'movement_finish_time_s': timeline['finish_time_s'],
            'start_time_s': 0.0, 'finish_time_s': end}


def plan_surface_support(tree, rows, grid, uuvs, cfg, end):
    """Plan midpoint support checkpoints with explicitly checked transit times.

    Anchor clearance to the complete UUV geometries makes long holds safe.
    Travel between anchors is checked against the scheduled UUVs in continuous
    time; moving earlier within the checkpoint interval is a bounded heuristic.
    """
    free = np.argwhere(np.asarray(rows) == 0)
    x0, y0 = grid['bounds'][:2]
    dx, dy = grid['dx_m'], grid['dy_m']
    centres = np.column_stack((x0+(free[:, 1]+.5)*dx, y0+(free[:, 0]+.5)*dy))
    index = cKDTree(centres)
    all_segments = [(np.asarray(s['p0']), np.asarray(s['p1']))
                    for u in uuvs for s in u['segments'] if s['p0'] != s['p1']]
    checkpoints = []
    for t in np.linspace(0, end, cfg['support_checkpoint_count']):
        midpoint = (np.asarray(position_at(uuvs[0], float(t)))+
                    np.asarray(position_at(uuvs[1], float(t))))/2
        _, indices = index.query(midpoint, k=min(20000, len(centres)))
        candidates = centres[indices]
        distance2 = np.full(len(candidates), np.inf)
        for a, b in all_segments:
            vector = b-a
            weights = np.clip((candidates-a)@vector/(vector@vector), 0, 1)
            delta = candidates-(a+weights[:, None]*vector)
            distance2 = np.minimum(distance2, np.einsum('ij,ij->i', delta, delta))
        allowed = np.flatnonzero(distance2 >= cfg['support_anchor_route_clearance_m']**2)
        if not len(allowed):
            raise ValueError('No safely separated surface support anchor near midpoint')
        chosen = int(allowed[0])
        checkpoints.append({'reference_time_s': float(t), 'midpoint_m': midpoint.tolist(),
                            'position_m': candidates[chosen].tolist(),
                            'snap_distance_m': float(np.linalg.norm(candidates[chosen]-midpoint)),
                            'all_uuv_route_clearance_m': float(math.sqrt(distance2[chosen]))})
    legs, pieces = [], []
    path = [checkpoints[0]['position_m']]
    previous_arrival = 0.0
    speed = cfg['platform_speeds_mps']['usv']
    decisions = []
    for checkpoint in checkpoints[1:]:
        a, b = path[-1], checkpoint['position_m']
        if a == b:
            checkpoint['actual_arrival_time_s'] = previous_arrival
            continue
        direct = tree.line_of_sight(a, b)
        connection = ([a, b] if direct else
                      shortcut(tree, astar(tree, a, b, parent_policy='theta_relaxation')['path']))
        connection = [list(p) for p in connection]
        length = sum(math.dist(p, q) for p, q in zip(connection, connection[1:]))
        leg = {'kind': 'support_transit', 'scanning': False,
               'points': connection, 'length_m': length,
               'connection_method': 'direct_LOS' if direct else 'theta_astar_then_LOS_shortcut'}
        duration = length/speed
        latest = checkpoint['reference_time_s']-duration
        if latest < previous_arrival-1e-9:
            raise ValueError('USV cannot reach support checkpoint at configured speed')
        # Latest departure first; use earlier slots only if an interaction fails.
        attempts = 0
        departure = latest
        while departure >= previous_arrival-1e-9:
            timed = time_parameterize([leg], speed, start_delay_s=max(previous_arrival, departure))
            distances = [pairwise_distance_extrema(timed, u, start_time_s=max(previous_arrival, departure),
                                                   end_time_s=timed['finish_time_s'])
                         for u in uuvs]
            attempts += 1
            if all(d['minimum_distance_m'] >= cfg['minimum_horizontal_separation_m']-1e-8
                   and d['maximum_distance_m'] <= cfg['support_range_m']+1e-8 for d in distances):
                break
            departure -= cfg['support_departure_step_s']
        else:
            raise ValueError('No separated USV departure slot within support checkpoint interval')
        if departure > previous_arrival+1e-9:
            pieces.append({'t0_s': previous_arrival, 't1_s': departure,
                           'p0': a, 'p1': a, 'kind': 'hold', 'scanning': False})
        pieces.extend(timed['segments'])
        legs.append(leg)
        path.extend(connection[1:])
        previous_arrival = timed['finish_time_s']
        checkpoint['actual_arrival_time_s'] = previous_arrival
        decisions.append({'departure_time_s': departure, 'arrival_time_s': previous_arrival,
                          'duration_s': duration, 'candidate_slots_checked': attempts})
    if previous_arrival < end:
        pieces.append({'t0_s': previous_arrival, 't1_s': end, 'p0': path[-1],
                       'p1': path[-1], 'kind': 'hold', 'scanning': False})
    length = sum(leg['length_m'] for leg in legs)
    timeline = {'segments': pieces, 'start_time_s': 0.0, 'finish_time_s': end,
                'clock_end_s': end, 'speed_m_s': speed, 'length_m': length,
                'initial_position': path[0], 'final_position': path[-1]}
    return {'role': 'surface_support', 'path': path, 'legs': legs, 'length_m': length,
            'scan_length_m': 0.0, 'transit_length_m': length, 'task_ids': [],
            'timeline': timeline, 'checkpoints': checkpoints,
            'departure_decisions': decisions}


def run(*, coverage_dir, prepared_dir, grid_dir, config, output_dir):
    cfg = json.loads(config.read_text())
    if (not all(math.isfinite(v) and v > 0 for v in cfg['platform_speeds_mps'].values())
            or cfg['support_checkpoint_count'] < 2
            or cfg['support_anchor_route_clearance_m'] < cfg['minimum_horizontal_separation_m']
            or cfg['support_range_m'] <= cfg['minimum_horizontal_separation_m']):
        raise ValueError('Invalid frozen cooperative experiment configuration')
    a2 = coverage_dir
    target_file, grid_file = prepared_dir/'searchable.geojson', grid_dir/'grid.json'
    input_checks = [(a2/'result.json', cfg['source_a2_result_sha256']),
                    (a2/'config.json', cfg['source_a2_config_sha256']),
                    (a2/'provenance.json', cfg['source_a2_provenance_sha256']),
                    (target_file, cfg['target_candidate_sha256']),
                    (grid_file, cfg['source_grid_sha256'])]
    for file, expected in input_checks:
        if digest(file) != expected:
            raise ValueError('Frozen scientific input changed: '+file.name)
    source = json.loads((a2/'result.json').read_text())
    provenance_a2 = json.loads((a2/'provenance.json').read_text())
    manifest_a1 = json.loads((prepared_dir/'manifest.json').read_text())
    source_cfg = json.loads((a2/'config.json').read_text())
    if source['status'] != 'geometric_coverage_passed':
        raise ValueError('A3 requires a passed A2 run')
    if (digest(target_file) != manifest_a1['derived_files_sha256']['searchable.geojson']
            or digest(prepared_dir/'config.json') != source_cfg['a1_config_sha256']
            or manifest_a1['config_sha256'] != source_cfg['a1_config_sha256']):
        raise ValueError('A1 source configuration or target manifest changed')
    if (digest(prepared_dir/'hazards.geojson') != source_cfg['a1_hazards_sha256']
            or manifest_a1['derived_files_sha256']['hazards.geojson'] != source_cfg['a1_hazards_sha256']):
        raise ValueError('Original hazard geometry changed')
    for name, expected in provenance_a2['geometry_sha256'].items():
        if digest(a2/name) != expected:
            raise ValueError('A2 footprint geometry changed: '+name)
    nav_file = a2/'navigation_grid.json'
    expected_nav_hash = next(value for key, value in provenance_a2['input_and_code_sha256'].items()
                             if key.endswith('/navigation_grid.json'))
    if digest(nav_file) != expected_nav_hash:
        raise ValueError('A2 navigation domain changed')
    nav = json.loads(nav_file.read_text())
    grid = json.loads(grid_file.read_text())
    rows = nav['rows_bottom_to_top']
    original = np.asarray(grid['rows_bottom_to_top'])
    if nav['bounds'] != grid['bounds'] or nav['n'] != grid['n'] or np.any((original != 0) & (np.asarray(rows) == 0)):
        raise ValueError('Navigation grid relaxed original obstacles')
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir/'config.json').write_bytes(config.read_bytes())
    start = time.perf_counter()
    tree = OccupancyQuadTree(grid['bounds'], rows)
    allocation = allocate_scan_tasks(tree, source['legs'])
    platforms = allocation['platforms']
    allocation['allocation']['task_owner'] = {task: key for key, platform in platforms.items()
                                             for task in platform['task_ids']}
    print(json.dumps({'phase': 'allocated', 'tasks': len(allocation['tasks']),
                      'routes_m': {k: v['length_m'] for k, v in platforms.items()}}), flush=True)
    reordered = plan_scan_tasks(tree, allocation['tasks'])
    nominal = [time_parameterize(platforms[key]['legs'], cfg['platform_speeds_mps'][key])
               for key in ('uuv_1', 'uuv_2')]
    departure = plan_departure_delays(nominal, cfg['minimum_horizontal_separation_m'],
                                    delay_step_s=cfg['uuv_departure_delay_step_s'],
                                    max_delay_s=cfg['maximum_uuv_departure_delay_s'])
    end = departure['mission_end_time_s']
    timelines = [complete_holds(t, end) for t in departure['timelines']]
    for key, timeline in zip(('uuv_1', 'uuv_2'), timelines):
        platforms[key].update({'role': 'coverage', 'timeline': timeline})
    platforms['usv'] = plan_surface_support(tree, rows, grid, timelines, cfg, end)
    print(json.dumps({'phase': 'timed', 'mission_end_s': end,
                      'departure_delays_s': departure['delays_s'],
                      'usv_length_m': platforms['usv']['length_m']}), flush=True)
    distances = {}
    for a, b in (('uuv_1', 'uuv_2'), ('usv', 'uuv_1'), ('usv', 'uuv_2')):
        distances[a+'__'+b] = pairwise_distance_extrema(platforms[a]['timeline'], platforms[b]['timeline'],
                                                       end_time_s=end)
    target = json.loads(target_file.read_text())['features'][0]['geometry']
    hazards = json.loads((prepared_dir/'hazards.geojson').read_text())['features'][0]['geometry']
    with Geos() as geos:
        path_checks = {}
        for key, platform in platforms.items():
            line = {'type': 'LineString', 'coordinates': platform['path']}
            inside = geos.covers(target, line)
            clearance = geos.distance(hazards, line)
            los = all(tree.line_of_sight(a, b) for a, b in zip(platform['path'], platform['path'][1:]))
            path_checks[key] = {'full_path_inside_candidate': inside,
                                'minimum_original_hazard_clearance_m': clearance,
                                'quadtree_LOS_all_segments': los,
                                'length_residual_m': abs(geos.measure(line)['length_m']-platform['length_m'])}
        control_line = {'type': 'LineString', 'coordinates': reordered['path']}
        control_check = geos.covers(target, control_line) and all(
            tree.line_of_sight(a, b) for a, b in zip(reordered['path'], reordered['path'][1:]))
        per_uuv_coverage = {}
        for key in ('uuv_1', 'uuv_2'):
            footprints = []
            for leg in platforms[key]['legs']:
                if leg['scanning']:
                    shape = geos.operation('buffer', {'type': 'LineString', 'coordinates': leg['points']},
                                           distance=source_cfg['swath_width_m']/2,
                                           quadrant_segments=source_cfg['footprint_quadrant_segments'])
                    footprints.append(geos.operation('intersection', shape, target))
            covered = geos.operation('union', {'type': 'GeometryCollection', 'geometries': footprints})
            area = geos.measure(covered)['area_m2']
            per_uuv_coverage[key] = {'covered_area_m2': area,
                                     'fraction_of_full_target': area/source['vector_coverage']['target_area_m2']}
            (output_dir/(key+'_covered.geojson')).write_text(json.dumps({
                'type': 'FeatureCollection', 'features': [{'type': 'Feature', 'properties': {},
                                                          'geometry': covered}]}, indent=2)+'\n')
        geos_version = geos.version
    for name in provenance_a2['geometry_sha256']:
        (output_dir/name).write_bytes((a2/name).read_bytes())
    sep_pass = all(d['minimum_distance_m'] >= cfg['minimum_horizontal_separation_m']-1e-8
                   for d in distances.values())
    range_pass = all(distances['usv__'+u]['maximum_distance_m'] <= cfg['support_range_m']+1e-8
                     for u in ('uuv_1', 'uuv_2'))
    geometry_pass = all(c['full_path_inside_candidate'] and c['quadtree_LOS_all_segments']
                        and c['minimum_original_hazard_clearance_m'] >= 50-1e-6
                        and c['length_residual_m'] < 1e-6 for c in path_checks.values())
    task_ids = [t for key in ('uuv_1', 'uuv_2') for t in platforms[key]['task_ids']]
    tasks_pass = (len(task_ids) == len(set(task_ids)) == len(allocation['tasks'])
                  and set(task_ids) == {t['task_id'] for t in allocation['tasks']})
    passed = sep_pass and range_pass and geometry_pass and tasks_pass and control_check
    comparison = {'a2_original_single_length_m': source['length_m'],
                  'a2_original_single_assumed_time_s': source['length_m']/cfg['platform_speeds_mps']['uuv_1'],
                  'reordered_single_length_m': reordered['length_m'],
                  'reordered_single_assumed_time_s': reordered['length_m']/cfg['platform_speeds_mps']['uuv_1'],
                  'two_uuv_total_length_m': sum(platforms[u]['length_m'] for u in ('uuv_1', 'uuv_2')),
                  'two_uuv_mission_end_s': end,
                  'two_uuv_vs_reordered_single_time_ratio': end/(reordered['length_m']/cfg['platform_speeds_mps']['uuv_1']),
                  'interpretation': 'Declared constant-speed geometry. Two UUVs use twice the coverage platforms; separate task reordering from parallelism. Deployment, recovery, dynamics and USV support effort are excluded from UUV-only length comparisons.'}
    result = {'status': 'cooperative_geometry_passed' if passed else 'cooperative_geometry_failed',
              'target_geometry_sha256': digest(target_file), 'tasks': allocation['tasks'],
              'allocation': allocation['allocation'], 'platforms': platforms,
              'mission_end_s': end, 'departure_schedule': departure,
              'pairwise_distances': distances, 'coverage': source['vector_coverage'],
              'per_uuv_coverage': per_uuv_coverage,
              'checks': {'tasks_exactly_once': tasks_pass, 'all_platform_paths': path_checks,
                         'continuous_separation_passed': sep_pass, 'continuous_support_range_passed': range_pass,
                         'reordered_single_control_geometry_passed': control_check},
              'comparison': comparison, 'clock_boundary': cfg['clock_boundary'],
              'limitations': cfg['limitations'], 'motion_constraints_verified': False,
              'three_dimensional_run': False, 'total_computation_seconds': time.perf_counter()-start}
    (output_dir/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (output_dir/'single_reordered_control.json').write_text(json.dumps(reordered, indent=2)+'\n')
    inputs = [config, a2/'result.json', a2/'config.json', a2/'provenance.json', nav_file,
              target_file, prepared_dir/'hazards.geojson', prepared_dir/'manifest.json',
              prepared_dir/'config.json', grid_file,
              grid_dir/'manifest.json']
    code = [Path(__file__), ROOT/'src/berthing_planner/cooperative_allocation.py',
            ROOT/'src/berthing_planner/mission_timing.py', ROOT/'src/berthing_planner/coverage.py',
            ROOT/'src/berthing_planner/occupancy.py', ROOT/'src/berthing_planner/search.py',
            ROOT/'src/berthing_planner/experiment/chart_geometry.py']
    snapshot = output_dir/'code_snapshot'; snapshot.mkdir()
    for file in code:
        destination=snapshot/file.relative_to(ROOT)
        destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes(file.read_bytes())
    provenance = {'recorded_utc': datetime.now(timezone.utc).isoformat(),
                  'source_a2_result_sha256': digest(a2/'result.json'),
                  'source_a2_config_sha256': digest(a2/'config.json'),
                  'target_candidate_sha256': digest(target_file), 'source_grid_sha256': digest(grid_file),
                  'input_and_code_sha256': {str(p): digest(p) for p in inputs+code},
                  'result_sha256': digest(output_dir/'result.json'),
                  'geometry_sha256': {p.name: digest(p) for p in output_dir.glob('*.geojson')},
                  'original_task_footprints_unchanged': True, 'GEOS_version': geos_version,
                  'raw_maps_uploaded': False, 'all_platforms_already_deployed_at_clock_start': True}
    (output_dir/'provenance.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'status': result['status'], 'mission_hours': end/3600, 'pairwise': distances,
                      'comparison': comparison, 'checks': result['checks']}, ensure_ascii=False, indent=2))
    if not passed:
        raise SystemExit(1)
