"""Independent source-to-schedule and whole-mission reference checks.

Uses circle-centre and phase integrals; no production clock, curve evaluator,
controller or distance solver is imported.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from ..experiment.chart_geometry import Geos
from .curves import IndependentClock, IndependentSchedule, IndependentWindow, arc_enclosure, audit_partition, piece_points, speed_checks


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_schedule(*, schedule_dir, routes_dir, scans_dir, config, json_out):
    read = lambda p: json.loads(p.read_text())
    x, route, repair, cfg = (read(p) for p in (schedule_dir/'result.json', routes_dir/'result.json', scans_dir/'result.json', config))
    proof = read(schedule_dir/'provenance.json')
    errors = []
    if digest(schedule_dir/'result.json') != proof['result_sha256'] or digest(schedule_dir/'uuv_pair.json') != proof['pair_sha256']:
        errors.append('schedule artifact hash')
    if any(digest(Path(p)) != h for p, h in proof['input_sha256'].items()):
        errors.append('schedule input hash')
    clocks = {n: IndependentClock(x['platforms'][n]) for n in ('uuv_1', 'uuv_2')}
    if abs(x['mission_end_s']-max(c.finish for c in clocks.values())) > 1e-7:
        errors.append('mission horizon')
    expected = []
    for name, clock in clocks.items():
        p = x['platforms'][name]
        source = route['platforms'][name]
        if p['pieces'] != source['pieces'] or p['profile'] != source['profile'] or p['source_task_ids'] != source['task_ids']:
            errors.append('UUV source curve/profile/tasks')
        offset = 0
        for index, leg in enumerate(source['legs']):
            count = len(leg['pieces'])
            if leg['scanning']:
                a, b = (source['profile']['segments'][i][key] for i, key in ((offset, 't0_s'), (offset+count-1, 't1_s')))
                expected.append(dict(task_id=leg['task_id'], platform=name, kind='source_scan', source_leg_index=index, local_start_s=a, local_end_s=b, global_start_s=a+clock.departure, global_end_s=b+clock.departure))
            offset += count
        if np.linalg.norm(clock.state(0)[0]-p['initial_pose'][:2]) > 1e-7 or np.linalg.norm(clock.state(clock.finish)[0]-p['final_pose'][:2]) > 1e-7:
            errors.append('UUV endpoint pose')
        holds = []
        if clock.departure > 0:
            holds.append(dict(t0_s=0., t1_s=clock.departure, pose=p['initial_pose'], kind='predeparture_hold', scanning=False))
        if clock.finish < x['mission_end_s']:
            holds.append(dict(t0_s=clock.finish, t1_s=x['mission_end_s'], pose=p['final_pose'], kind='completed_hold', scanning=False))
        if p['holds'] != holds:
            errors.append('explicit UUV holds')
    for r in repair['selected_repairs']:
        n = r['platform']; a, b = r['local_start_s'], r['local_end_s']
        expected.append(dict(task_id=r['task_id'], platform=n, kind='repair_scan', source_leg_index=r['source_transit_leg_index'], local_start_s=a, local_end_s=b, global_start_s=a+clocks[n].departure, global_end_s=b+clocks[n].departure))
    expected.sort(key=lambda w: (w['platform'], w['global_start_s'], w['task_id']))
    if expected != x['scan_windows'] or len(expected) != 89 or not repair['coverage_gate_passed'] or x['coverage_lower_ratio_unchanged'] != repair['coverage_lower_ratio']:
        errors.append('complete source and repair scan accounting')
    surface = IndependentClock(route['platforms']['usv'])
    initial = x['platforms']['usv']['initial_pose']
    if np.linalg.norm(surface.state(0)[0]-initial[:2]) > 1e-7 or abs(math.remainder(surface.state(0)[3]-initial[2], 2*math.pi)) > 1e-8:
        errors.append('USV initial pose')
    if x['platforms']['uuv_1']['departure_s'] != 0 or x['platforms']['uuv_2']['departure_s'] not in cfg['uuv2_departure_candidates_s']:
        errors.append('departure candidate set')
    cert = read(schedule_dir/'uuv_pair.json')
    check = audit_partition(cert, clocks['uuv_1'], clocks['uuv_2'], x['mission_end_s'])
    errors.extend(check['errors'])
    if not cert['converged'] or cert['minimum_distance_lower_m'] < cfg['minimum_separation_m']:
        errors.append('whole-time UUV separation')
    report = dict(passed=not errors, errors=errors, source_result_sha256=digest(schedule_dir/'result.json'), scan_count=len(expected), distance_audit=check)
    json_out.write_text(json.dumps(report, indent=2)+'\n')
    if errors:
        raise ValueError('Schedule audit: '+', '.join(sorted(set(errors))))


def run(*, reference_dir, schedule_dir, prepared_dir, navigation_dir, json_out):
    read = lambda p: json.loads(p.read_text())
    x, seed, cfg, proof = (read(p) for p in (reference_dir/'result.json', schedule_dir/'result.json', reference_dir/'config.json', reference_dir/'provenance.json'))
    seed_audit = read(schedule_dir/'audit.json')
    errors = []
    if not seed_audit['passed'] or seed_audit['source_result_sha256'] != digest(schedule_dir/'result.json'):
        errors.append('matching schedule audit')
    if digest(reference_dir/'result.json') != proof['result_sha256'] or any(digest(Path(p)) != h for p, h in proof['input_sha256'].items()) or any(digest(reference_dir/p) != h for p, h in proof['derived_sha256'].items()):
        errors.append('reference artifact/input hash')
    if x['scan_windows'] != seed['scan_windows'] or x['mission_end_s'] != seed['mission_end_s'] or x['coverage_lower_ratio_unchanged'] != seed['coverage_lower_ratio_unchanged']:
        errors.append('schedule inheritance')
    if any(x['uuv_platforms'][n] != seed['platforms'][n] for n in x['uuv_platforms']) or set(x['uuv_platforms']) != {'uuv_1', 'uuv_2'}:
        errors.append('UUV inheritance')
    if x['mobile_usv']['initial_pose'] != seed['platforms']['usv']['initial_pose']:
        errors.append('USV initial pose')
    clocks = {n: IndependentClock(p) for n, p in x['uuv_platforms'].items()}
    surface = IndependentSchedule(x['mobile_usv']['legs'], x['mobile_usv']['initial_pose'])
    if surface.join_error > 1e-5:
        errors.append('USV position joins')
    shapes = {}
    for name, key, directory in (('navigation', 'navigation_sha256', navigation_dir), ('hazards', 'hazards_geometry_sha256', prepared_dir), ('searchable', 'target_geometry_sha256', prepared_dir)):
        path = directory/(name+'.geojson')
        if digest(path) != cfg[key]:
            errors.append('chart pin '+name)
        shapes[name] = read(path)['features'][0]['geometry']
    clearance = math.inf
    with Geos() as g:
        previous = x['mobile_usv']['initial_pose']; finish = 0.; holds = []
        for leg, clock in zip(x['mobile_usv']['legs'], surface.clocks):
            errors.extend(speed_checks(leg['profile'], leg['pieces'], dict(cruise_mps=cfg['usv_cruise_mps'], tangential_accel_mps2=cfg['tangential_accel_mps2'], lateral_accel_mps2=cfg['lateral_accel_mps2'])))
            if clock.departure < finish or clock.finish > x['mission_end_s']:
                errors.append('USV schedule order/horizon')
            if clock.departure > finish:
                holds.append(dict(t0_s=finish, t1_s=clock.departure, pose=list(previous), scanning=False))
            first = clock.pieces[0]['pose']
            if np.linalg.norm(np.asarray(previous[:2])-first[:2]) > 1e-6 or abs(math.remainder(previous[2]-first[2], 2*math.pi)) > 1e-8:
                errors.append('USV leg pose join')
            for a, b in zip(clock.pieces, clock.pieces[1:]):
                if np.linalg.norm(piece_points(a, [1])[0]-b['pose'][:2]) > 1e-6 or abs(math.remainder(a['pose'][2]+a['curvature']*a['length']-b['pose'][2], 2*math.pi)) > 1e-8:
                    errors.append('USV primitive pose join')
            if any(p['kind'] not in ('line', 'arc') or p['direction'] != 1 or abs(p['curvature']) > 1/cfg['usv_radius_m']+1e-12 for p in clock.pieces):
                errors.append('forward/radius constraint')
            points, error = arc_enclosure(clock.pieces, .005)
            shape = dict(type='LineString', coordinates=points)
            envelope = g.operation('buffer', shape, distance=(error+1e-6)/math.cos(math.pi/480), quadrant_segments=120)
            lo = g.distance(shape, shapes['hazards'])-error-1e-6
            clearance = min(clearance, lo)
            if not g.covers(shapes['navigation'], envelope) or lo < cfg['hazard_margin_m']:
                errors.append('USV navigation/hazard enclosure')
            finish = clock.finish
            last = clock.pieces[-1]
            previous = [*piece_points(last, [1])[0], last['pose'][2]+last['curvature']*last['length']]
        if finish < x['mission_end_s']:
            holds.append(dict(t0_s=finish, t1_s=x['mission_end_s'], pose=list(previous), scanning=False))
        saved = x['mobile_usv']['holds']
        if len(saved) != len(holds) or any(a['t0_s'] != b['t0_s'] or a['t1_s'] != b['t1_s'] or a['scanning'] or np.linalg.norm(np.asarray(a['pose'][:2])-b['pose'][:2]) > 1e-6 or abs(math.remainder(a['pose'][2]-b['pose'][2], 2*math.pi)) > 1e-8 for a, b in zip(saved, holds)):
            errors.append('explicit USV holds')
    pairs = x['distance_pairs']; checks = {}
    clocks['usv'] = surface
    if set(pairs) != {'usv__uuv_1', 'usv__uuv_2', 'uuv_1__uuv_2'}:
        raise ValueError('All three distance pairs required')
    for name, summary in pairs.items():
        a, b = name.split('__'); cert = read(reference_dir/summary['certificate_file'])
        check = audit_partition(cert, clocks[a], clocks[b], x['mission_end_s']); checks[name] = check
        errors.extend(check['errors'])
        if {k: v for k, v in cert.items() if k != 'partition'} != {k: v for k, v in summary.items() if k != 'certificate_file'}:
            errors.append('distance summary binding')
        if not cert['converged'] or cert['minimum_distance_lower_m'] < cfg['minimum_separation_m'] or (a == 'usv' and cert['maximum_distance_upper_m'] > cfg['mobile_design_range_m']):
            errors.append('whole-mission distance gate')
    # Audit every selected window as well as the full mission. Candidate samples
    # and rejected historical options never substitute for these certificates.
    decisions = x['window_decisions']; prior = 0.
    for row in decisions:
        a, b = row['t0_s'], row['t1_s']
        if a != prior or b <= a:
            errors.append('support window partition')
        for n in ('uuv_1', 'uuv_2'):
            cert = read(reference_dir/f"certificates/window_{row['index']:02d}_selected_{n}.json")
            check = audit_partition(cert, IndependentWindow(surface, a), IndependentWindow(clocks[n], a), b-a)
            errors.extend(check['errors'])
            if not cert['converged'] or cert['minimum_distance_lower_m'] < cfg['minimum_separation_m'] or cert['maximum_distance_upper_m'] > cfg['mobile_design_range_m']:
                errors.append('selected support window gate')
        prior = b
    if prior != x['mission_end_s'] or abs(sum(c.profile['length_m'] for c in surface.clocks)-x['mobile_usv']['length_m']) > 1e-6:
        errors.append('complete support horizon/length')
    report = dict(passed=not errors, errors=sorted(set(errors)), source_result_sha256=digest(reference_dir/'result.json'), scan_count=len(x['scan_windows']), distance_audits=checks, independent_usv_hazard_clearance_lower_m=clearance, scope='Fixed full-target planar cooperative reference; finite support search, continuous interval audit.')
    json_out.write_text(json.dumps(report, indent=2)+'\n')
    if errors:
        raise ValueError('Reference audit: '+', '.join(sorted(set(errors))))
