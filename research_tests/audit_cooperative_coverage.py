"""Independent A3 task, affine-time, raw-grid and coverage-inheritance audit.

No imports from the production allocation or mission-timing implementation.
Coverage inheritance is proved by identical active tasks and A2 footprint files;
this audit does not use GEOS to recompute the producer's footprint areas.
"""
import argparse
from collections import Counter
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np

from audit_open_chart_coverage import closed_segment_blocked_cells, route_checks

_PLATFORMS = ('usv', 'uuv_1', 'uuv_2')


def _segments(platform):
    timeline = platform['timeline']
    return timeline['segments'] if isinstance(timeline, dict) else timeline


def _xy(values):
    points = np.asarray(values, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        raise ValueError('Expected nonempty finite XY points')
    return points


def _same_segment(first, second):
    a, b = _xy(first), _xy(second)
    ends_match = (np.all(np.abs(a[[0, -1]]-b[[0, -1]]) <= 1e-7)
                  or np.all(np.abs(a[[0, -1]]-b[[-1, 0]]) <= 1e-7))
    chord_a, chord_b = np.linalg.norm(a[-1]-a[0]), np.linalg.norm(b[-1]-b[0])
    length_a = np.linalg.norm(np.diff(a, axis=0), axis=1).sum()
    length_b = np.linalg.norm(np.diff(b, axis=0), axis=1).sum()
    return bool(ends_match and abs(length_a-chord_a) <= 1e-7 and abs(length_b-chord_b) <= 1e-7)


def task_accounting(result, a2_result):
    expected_sources = {i: leg for i, leg in enumerate(a2_result['legs'])
                        if leg['scanning'] and leg['kind'] != 'transit'}
    tasks = result['tasks']
    task_ids = [t['task_id'] for t in tasks]
    if any(not isinstance(i, str) or not i for i in task_ids):
        raise ValueError('Task IDs must be nonempty strings')
    by_id = {t['task_id']: t for t in tasks}
    sources = [t['source_leg_index'] for t in tasks]
    sources_ok = Counter(sources) == Counter(expected_sources.keys())
    unique_ids = len(set(task_ids)) == len(task_ids)
    source_geometry_errors, source_length_errors = [], []
    for task in tasks:
        source = expected_sources.get(task['source_leg_index'])
        if source is None:
            source_geometry_errors.append(task['task_id'])
            continue
        if task['kind'] != source['kind'] or not _same_segment(task['points'], source['points']):
            source_geometry_errors.append(task['task_id'])
        actual = float(np.linalg.norm(np.diff(_xy(task['points']), axis=0), axis=1).sum())
        if abs(actual-task['length_m']) > 1e-6 or abs(actual-source['length_m']) > 1e-6:
            source_length_errors.append(task['task_id'])
    owners, declared_ids, leg_ids, timeline_ids = {}, [], [], []
    assignment_errors, leg_geometry_errors, timeline_geometry_errors, illegal_scan = [], [], [], []
    for name in _PLATFORMS:
        platform = result['platforms'][name]
        assigned = platform.get('task_ids', [])
        declared_ids.extend(assigned)
        active_legs = [leg for leg in platform['legs'] if leg['scanning']]
        active_timeline = [s for s in _segments(platform) if s['scanning']]
        if name == 'usv':
            if assigned or active_legs or active_timeline:
                illegal_scan.append(name)
            continue
        active_ids = [leg.get('task_id') for leg in active_legs]
        timed_ids = [s.get('task_id') for s in active_timeline]
        leg_ids.extend(active_ids)
        timeline_ids.extend(timed_ids)
        if Counter(assigned) != Counter(active_ids) or Counter(assigned) != Counter(timed_ids):
            assignment_errors.append(name)
        for task_id in assigned:
            if task_id in owners:
                assignment_errors.append(task_id)
            owners[task_id] = name
        for leg in active_legs:
            task = by_id.get(leg.get('task_id'))
            if (task is None or leg['kind'] not in ('sweep', 'supplement')
                    or leg['kind'] != task['kind'] or not _same_segment(leg['points'], task['points'])):
                leg_geometry_errors.append(leg.get('task_id'))
        for segment in active_timeline:
            task = by_id.get(segment.get('task_id'))
            if (task is None or segment['kind'] not in ('sweep', 'supplement')
                    or segment['kind'] != task['kind']
                    or not _same_segment([segment['p0'], segment['p1']], task['points'])):
                timeline_geometry_errors.append(segment.get('task_id'))
    one_each = Counter(declared_ids) == Counter(task_ids) == Counter(leg_ids) == Counter(timeline_ids)
    allocation_matches = result['allocation']['task_owner'] == owners
    check = {'A2_active_task_count': len(expected_sources), 'declared_task_count': len(tasks),
             'source_leg_indices_exactly_once': sources_ok, 'task_ids_unique': unique_ids,
             'source_geometry_error_task_ids': source_geometry_errors,
             'source_length_error_task_ids': source_length_errors,
             'assignment_errors': assignment_errors,
             'active_leg_geometry_error_task_ids': leg_geometry_errors,
             'active_timeline_geometry_error_task_ids': timeline_geometry_errors,
             'USV_scan_or_assignment_errors': illegal_scan,
             'declared_legs_and_timeline_execute_each_task_once': one_each,
             'allocation_owner_matches_execution': allocation_matches}
    check['passed'] = (sources_ok and unique_ids and one_each and allocation_matches
                       and not source_geometry_errors and not source_length_errors
                       and not assignment_errors and not leg_geometry_errors
                       and not timeline_geometry_errors and not illegal_scan)
    return check


def timeline_checks(platform, speed_mps, mission_end_s):
    segments = _segments(platform)
    if not segments or not speed_mps > 0 or not mission_end_s > 0:
        raise ValueError('Positive speed/horizon and a nonempty timeline are required')
    path = _xy(platform['path'])
    gaps, jumps, speed_errors, bad_duration, bad_holds = [], [], [], [], []
    outside, moving_intervals, points = [], [], []
    previous_time, previous_point = 0.0, path[0]
    for index, segment in enumerate(segments):
        t0, t1 = float(segment['t0_s']), float(segment['t1_s'])
        p0, p1 = _xy([segment['p0'], segment['p1']])
        if not math.isfinite(t0) or not math.isfinite(t1) or not isinstance(segment['scanning'], bool):
            raise ValueError('Finite timeline times and boolean scanning flags are required')
        gaps.append(abs(t0-previous_time))
        jumps.append(float(np.linalg.norm(p0-previous_point)))
        duration, distance = t1-t0, float(np.linalg.norm(p1-p0))
        if duration < 0 or (duration == 0 and distance > 1e-7):
            bad_duration.append(index)
        if t0 < -1e-8 or t1 > mission_end_s+1e-8:
            outside.append(index)
        if segment['kind'] == 'hold':
            if distance > 1e-7 or segment['scanning']:
                bad_holds.append(index)
        elif segment['kind'] not in ('sweep', 'supplement', 'transit', 'support_transit'):
            bad_duration.append(index)
        elif duration > 0:
            speed_errors.append(abs(distance/duration-speed_mps))
            moving_intervals.append((t0, t1))
        if not points:
            points.append(p0)
        points.append(p1)
        previous_time, previous_point = t1, p1
    geometry = np.asarray(points)
    geometry = geometry[np.r_[True, np.linalg.norm(np.diff(geometry, axis=0), axis=1) > 1e-7]]
    clean_path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-7]]
    same = geometry.shape == clean_path.shape and bool(np.all(np.abs(geometry-clean_path) <= 1e-7))
    final_time_error = abs(previous_time-mission_end_s)
    final_position_error = float(np.linalg.norm(previous_point-path[-1]))
    start_move = min((a for a, _ in moving_intervals), default=mission_end_s)
    finish_move = max((b for _, b in moving_intervals), default=0)
    header_errors = []
    if isinstance(platform['timeline'], dict):
        expected = {'start_time_s': 0, 'finish_time_s': mission_end_s,
                    'clock_end_s': mission_end_s, 'speed_m_s': speed_mps,
                    'length_m': platform['length_m'], 'movement_start_time_s': start_move,
                    'movement_finish_time_s': finish_move}
        header = platform['timeline']
        header_errors = [key for key, value in expected.items()
                         if key in header and abs(header[key]-value) > 1e-6]
        for key, value in (('initial_position', path[0]), ('final_position', path[-1])):
            if key in header and np.linalg.norm(np.asarray(header[key])-value) > 1e-7:
                header_errors.append(key)
    check = {'covers_full_mission_clock': max(gaps, default=0) <= 1e-8 and final_time_error <= 1e-8,
             'maximum_time_gap_or_overlap_s': max(gaps, default=0),
             'final_time_error_s': final_time_error, 'maximum_position_jump_m': max(jumps, default=0),
             'final_position_error_m': final_position_error,
             'maximum_constant_speed_error_mps': max(speed_errors, default=0),
             'bad_duration_segment_indices': bad_duration, 'bad_hold_segment_indices': bad_holds,
             'segments_outside_mission_clock': outside, 'timeline_geometry_matches_path': same,
             'timeline_header_errors': header_errors,
             'first_movement_start_s': start_move, 'last_movement_end_s': finish_move,
             'initial_hold_duration_s': start_move, 'final_hold_duration_s': mission_end_s-finish_move}
    check['passed'] = (check['covers_full_mission_clock'] and same and not bad_duration and not bad_holds
                       and not outside and not header_errors and check['maximum_position_jump_m'] <= 1e-7
                       and final_position_error <= 1e-7 and check['maximum_constant_speed_error_mps'] <= 1e-6)
    return check


def continuous_pair_extrema(first, second, mission_end_s):
    """Analytic distances on every common piecewise-affine time interval.

    Squared relative distance is q(t)=|r+v*t|². Its minimum is the clipped
    stationary point; its maximum is at an interval endpoint. This uses all
    timeline breakpoints, not replay snapshots. Input timelines are validated
    by timeline_checks before this scientific check is called.
    """
    a = [s for s in first if s['t1_s'] > s['t0_s']]
    b = [s for s in second if s['t1_s'] > s['t0_s']]
    times = sorted({0.0, mission_end_s, *(s[k] for s in a+b for k in ('t0_s', 't1_s'))})
    i = j = 0
    minimum, maximum, min_time, max_time = math.inf, -math.inf, None, None
    min_interval = max_interval = None
    for lo, hi in zip(times, times[1:]):
        if hi <= lo:
            continue
        while i+1 < len(a) and a[i]['t1_s'] <= lo:
            i += 1
        while j+1 < len(b) and b[j]['t1_s'] <= lo:
            j += 1
        sa, sb = a[i], b[j]
        va = (np.asarray(sa['p1'])-sa['p0'])/(sa['t1_s']-sa['t0_s'])
        vb = (np.asarray(sb['p1'])-sb['p0'])/(sb['t1_s']-sb['t0_s'])
        pa = np.asarray(sa['p0'])+va*(lo-sa['t0_s'])
        pb = np.asarray(sb['p0'])+vb*(lo-sb['t0_s'])
        r, v, duration = pa-pb, va-vb, hi-lo
        quadratic = float(v@v)
        candidates = [0.0, duration]
        if quadratic > 0:
            candidates.append(float(np.clip(-float(r@v)/quadratic, 0, duration)))
        for t in candidates:
            distance = float(np.linalg.norm(r+v*t))
            if distance < minimum:
                minimum, min_time, min_interval = distance, lo+t, [lo, hi]
            if distance > maximum:
                maximum, max_time, max_interval = distance, lo+t, [lo, hi]
    return {'minimum_distance_m': minimum, 'minimum_at_s': min_time, 'minimum_interval_s': min_interval,
            'maximum_distance_m': maximum, 'maximum_at_s': max_time, 'maximum_interval_s': max_interval,
            'common_affine_intervals_checked': len(times)-1}


def audit_data(result, a2_result, config, blocked, bounds):
    if set(result['platforms']) != set(_PLATFORMS):
        raise ValueError('Expected exactly USV, UUV 1 and UUV 2')
    mission_end = result['mission_end_s']
    geometry, clocks, collisions = {}, {}, {}
    for name in _PLATFORMS:
        platform = result['platforms'][name]
        view = {**platform, 'legs': [{**leg, 'kind': 'transit' if leg['kind'] == 'support_transit'
                                     else leg['kind']} for leg in platform['legs']]}
        if not platform['legs'] and len(platform['path']) == 1:
            geometry[name] = {'stationary_single_point_path': True,
                              'passed': abs(platform['length_m']) <= 1e-6}
        else:
            geometry[name] = route_checks(view)
        clocks[name] = timeline_checks(platform, config['platform_speeds_mps'][name], mission_end)
        path = _xy(platform['path'])
        outside = int(np.sum((path[:, 0] < bounds[0]) | (path[:, 0] > bounds[2]) |
                             (path[:, 1] < bounds[1]) | (path[:, 1] > bounds[3])))
        examples, hit_count = [], 0
        pairs = zip(path, path[1:]) if len(path) > 1 else [(path[0], path[0])]
        for index, (a, b) in enumerate(pairs):
            cells = closed_segment_blocked_cells(a, b, blocked, bounds)
            if len(cells):
                hit_count += 1
                if len(examples) < 10:
                    examples.append({'segment_index': index, 'cells_row_col': cells[:10].tolist()})
        collisions[name] = {'points_outside_grid': outside, 'segments_hitting_raw_blocked_cells': hit_count,
                            'examples': examples, 'passed': outside == hit_count == 0}
    tasks = task_accounting(result, a2_result)
    roles = (result['platforms']['usv']['role'] == 'surface_support'
             and all(result['platforms'][n]['role'] == 'coverage' for n in ('uuv_1', 'uuv_2')))
    distances = {}
    if all(check['passed'] for check in clocks.values()):
        for first, second in itertools.combinations(_PLATFORMS, 2):
            check = continuous_pair_extrema(_segments(result['platforms'][first]),
                                             _segments(result['platforms'][second]), mission_end)
            check['separation_passed'] = check['minimum_distance_m']+1e-7 >= config['minimum_horizontal_separation_m']
            check['support_range_applicable'] = first == 'usv'
            check['support_range_passed'] = (not check['support_range_applicable']
                                             or check['maximum_distance_m'] <= config['support_range_m']+1e-7)
            distances[first+'__'+second] = check
    covered = result['coverage'] == a2_result['vector_coverage']
    passed = (result['status'] == 'cooperative_geometry_passed' and roles and tasks['passed'] and covered
              and all(check['passed'] for check in geometry.values())
              and all(check['passed'] for check in clocks.values())
              and all(check['passed'] for check in collisions.values()) and len(distances) == 3
              and all(d['separation_passed'] and d['support_range_passed'] for d in distances.values()))
    return {'passed': passed, 'roles_correct': roles, 'mission_end_s': mission_end,
            'tasks': tasks, 'route_geometry': geometry, 'timeline': clocks,
            'raw_grid_collision': collisions, 'continuous_pair_distances': distances,
            'coverage_metrics_identical_to_A2': covered,
            'coverage_inheritance_reason': 'Each unchanged A2 active footprint task is scanned exactly once by a coverage platform; transit/holds/USV do not scan; task reversal preserves the ideal footprint set.'}


def audit_run(run_dir, a2_run_dir, prepared_dir, grid_dir):
    files = {'A3_result': run_dir/'result.json', 'A3_config': run_dir/'config.json',
             'A3_provenance': run_dir/'provenance.json', 'A2_result': a2_run_dir/'result.json',
             'A2_config': a2_run_dir/'config.json', 'A2_provenance': a2_run_dir/'provenance.json',
             'A1_target': prepared_dir/'searchable.geojson', 'A1_manifest': prepared_dir/'manifest.json',
             'A1_summary': prepared_dir/'summary.json', 'raw_grid': grid_dir/'grid.json',
             'raw_grid_manifest': grid_dir/'manifest.json'}
    data = {key: json.loads(file.read_text()) for key, file in files.items()}
    hashes = {key: hashlib.sha256(file.read_bytes()).hexdigest() for key, file in files.items()}
    result, previous, config, grid = data['A3_result'], data['A2_result'], data['A3_config'], data['raw_grid']
    target_hash = hashes['A1_target']
    target_unchanged = (target_hash == data['A1_manifest']['derived_files_sha256']['searchable.geojson']
                        == result['target_geometry_sha256'] == previous['target_geometry_sha256']
                        == result['coverage']['target_geometry_sha256'])
    target_area = data['A1_summary']['searchable_area_m2']
    full_target_area = abs(result['coverage']['target_area_m2']-target_area) <= 1e-6
    source_grid_unchanged = hashes['raw_grid'] == data['raw_grid_manifest']['grid_sha256']
    a2_integrity = hashes['A2_result'] == data['A2_provenance']['result_sha256']
    footprint_files = {}
    for name in ('covered.geojson', 'missed.geojson', 'repeated.geojson', 'initial_missed.geojson'):
        old, new = a2_run_dir/name, run_dir/name
        old_hash = hashlib.sha256(old.read_bytes()).hexdigest()
        new_hash = hashlib.sha256(new.read_bytes()).hexdigest()
        footprint_files[name] = {'A2_sha256': old_hash, 'A3_sha256': new_hash,
                                 'unchanged': old_hash == new_hash == data['A2_provenance']['geometry_sha256'][name]
                                 == data['A3_provenance']['geometry_sha256'][name]}
    provenance = data['A3_provenance']
    recorded_inputs = set(provenance.get('input_sha256', provenance.get('input_and_code_sha256', {})).values())
    required_input_keys = ('A2_result', 'A2_config', 'A2_provenance', 'A1_target', 'raw_grid', 'raw_grid_manifest')
    provenance_matches = all(hashes[key] in recorded_inputs for key in required_input_keys)
    result_matches = hashes['A3_result'] == provenance['result_sha256']
    swath_same = config.get('swath_width_m', data['A2_config']['swath_width_m']) == data['A2_config']['swath_width_m']
    blocked = np.asarray(grid['rows_bottom_to_top'], dtype=bool)
    if blocked.shape != (grid['n'], grid['n']):
        raise ValueError('Raw grid shape differs from declared n')
    audit = audit_data(result, previous, config, blocked, grid['bounds'])
    audit.update(target_geometry_unchanged=target_unchanged, full_original_target_area_retained=full_target_area,
                 raw_original_grid_sha256_verified=source_grid_unchanged, A2_result_sha256_verified=a2_integrity,
                 coverage_files=footprint_files, input_provenance_matches=provenance_matches,
                 A3_result_sha256_verified=result_matches, swath_width_unchanged=swath_same,
                 input_sha256=hashes, audit_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                 reused_independent_grid_auditor_sha256=hashlib.sha256(
                     Path(__file__).with_name('audit_open_chart_coverage.py').read_bytes()).hexdigest(),
                 limits='Piecewise-affine 2D point-platform geometry with declared constant speeds and ideal unoccluded scan footprints. Initial deployment before clock and recovery after clock are excluded. No curvature, acceleration, turning dynamics, actual depth, acoustic connectivity, obstacle occlusion, hull, control or sea-trial claim.')
    audit['passed'] = (audit['passed'] and target_unchanged and full_target_area and source_grid_unchanged
                       and a2_integrity and provenance_matches and result_matches and swath_same
                       and all(f['unchanged'] for f in footprint_files.values()))
    return audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('run-dir', 'a2-run-dir', 'prepared-dir', 'grid-dir', 'json-out'):
        parser.add_argument('--'+key, type=Path, required=True)
    args = parser.parse_args()
    if args.json_out.exists():
        raise FileExistsError('Preserving the preceding audit; choose a new JSON output')
    audit = audit_run(args.run_dir, args.a2_run_dir, args.prepared_dir, args.grid_dir)
    args.json_out.write_text(json.dumps(audit, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    if not audit['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
