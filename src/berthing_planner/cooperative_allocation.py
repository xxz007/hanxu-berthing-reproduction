"""Deterministic two-platform scan-task bands with scan-off safe connectors.

Tasks retain the A2 scanning geometry; only order and endpoint orientation may
change. Midpoint-y bands divide tasks, not necessarily disjoint footprint areas.
Deployment at each first scan point and recovery after the route are excluded.
"""
from itertools import groupby
import math

from .coverage import append_sweep


def _normalise_tasks(tasks):
    """Validate task dictionaries once at the public boundary."""
    result, identifiers = [], set()
    for task in tasks:
        identifier = task['task_id']
        if not isinstance(identifier, str) or not identifier or identifier in identifiers:
            raise ValueError('Scan task IDs must be unique nonempty strings')
        if task['kind'] not in ('sweep', 'supplement'):
            raise ValueError('Only sweep or supplement tasks may be allocated')
        points = task['points']
        if len(points) != 2 or any(len(point) != 2 for point in points):
            raise ValueError('Every active task must be one straight segment')
        if not all(math.isfinite(value) for point in points for value in point):
            raise ValueError('Task endpoints must be finite')
        length = task['length_m']
        if not math.isfinite(length) or length < 0 or not math.isclose(
                length, math.dist(*points), rel_tol=1e-12, abs_tol=1e-6):
            raise ValueError('Task length must match its straight endpoints')
        if not isinstance(task['source_leg_index'], int) or task['source_leg_index'] < 0:
            raise ValueError('Source leg index must be a nonnegative integer')
        result.append({'task_id': identifier, 'source_leg_index': task['source_leg_index'],
                       'kind': task['kind'], 'points': [list(point) for point in points],
                       'length_m': length})
        identifiers.add(identifier)
    return result


def _ordered_tasks(tasks):
    """Sort midpoint rows south-to-north; alternate x order and orientation."""
    sorted_tasks = sorted(tasks, key=lambda task: (
        (task['points'][0][1]+task['points'][1][1])/2,
        (task['points'][0][0]+task['points'][1][0])/2, task['task_id']))
    ordered = []
    for row_index, (_, group) in enumerate(groupby(
            sorted_tasks, key=lambda task: (task['points'][0][1]+task['points'][1][1])/2)):
        row = list(group)
        if row_index % 2:
            row.reverse()
        for task in row:
            a, b = sorted(task['points'])
            if row_index % 2:
                a, b = b, a
            ordered.append((task, a, b))
    return ordered


def _estimated_length(tasks):
    ordered = _ordered_tasks(tasks)
    return sum(task['length_m'] for task in tasks) + sum(
        math.dist(previous[2], following[1]) for previous, following in zip(ordered, ordered[1:]))


def _plan_scan_tasks(tree, tasks):
    plan = {'path': [], 'legs': [], 'length_m': 0.0, 'scan_length_m': 0.0,
            'transit_length_m': 0.0, 'supplement_length_m': 0.0,
            'scan_leg_count': 0, 'direction_change_count': 0, 'task_ids': [],
            'direction_change_definition': 'heading changes in nonzero route segments, not propulsion reversals',
            'task_ordering': 'midpoint y ascending; equal-y x groups serpentine; endpoint reversal allowed',
            'deployment_model': 'deployed at first scan point; launch transfer and recovery are excluded'}
    for task, a, b in _ordered_tasks(tasks):
        append_sweep(tree, plan, a, b, kind=task['kind'])
        plan['legs'][-1]['task_id'] = task['task_id']
        plan['task_ids'].append(task['task_id'])
    plan['deployed_start'] = list(plan['path'][0]) if plan['path'] else None
    plan['route_end'] = list(plan['path'][-1]) if plan['path'] else None
    return plan


def plan_scan_tasks(tree, tasks):
    """Single-platform reordered control, using the same ordering as each band.

    Tasks use the dictionaries returned by allocate_scan_tasks. An empty input
    yields an explicitly empty route, with no implicit deployment movement.
    """
    return _plan_scan_tasks(tree, _normalise_tasks(tasks))


def allocate_scan_tasks(tree, source_legs, platform_ids=('uuv_1', 'uuv_2')):
    """Allocate all source scan legs exactly once to two midpoint-y bands.

    Evaluate nonempty cuts that keep equal-y task groups together when any
    such cut exists. Otherwise allow an explicitly reported same-y split.
    Minimise the maximum of scan length plus straight connector estimates;
    resolve ties by estimated imbalance, then cut index. This is a heuristic,
    not an obstacle-aware or time-optimal assignment. Actual legs use safe
    LOS/A* connectors and may therefore differ from the estimated workload.
    """
    platform_ids = tuple(platform_ids)
    if len(platform_ids) != 2 or len(set(platform_ids)) != 2 or not all(
            isinstance(identifier, str) and identifier for identifier in platform_ids):
        raise ValueError('Exactly two distinct nonempty platform IDs are required')
    tasks = []
    for index, leg in enumerate(source_legs):
        if leg['kind'] == 'transit':
            if leg['scanning'] is not False:
                raise ValueError('Source transit legs must have scanning=False')
            continue
        if leg['scanning'] is not True:
            raise ValueError('Source scan legs must have scanning=True')
        tasks.append({'task_id': f'scan_{len(tasks):03d}', 'source_leg_index': index,
                      'kind': leg['kind'], 'points': leg['points'], 'length_m': leg['length_m']})
    tasks = _normalise_tasks(tasks)
    if len(tasks) < 2:
        raise ValueError('Two active platforms require at least two scan tasks')
    bands = sorted(tasks, key=lambda task: (
        (task['points'][0][1]+task['points'][1][1])/2,
        (task['points'][0][0]+task['points'][1][0])/2, task['task_id']))
    midpoints_y = [(task['points'][0][1]+task['points'][1][1])/2 for task in bands]
    cuts = [index for index in range(1, len(bands)) if midpoints_y[index-1] != midpoints_y[index]]
    grouped = bool(cuts)
    if not cuts:
        cuts = list(range(1, len(bands)))
    candidates = []
    for cut in cuts:
        south, north = _estimated_length(bands[:cut]), _estimated_length(bands[cut:])
        candidates.append((max(south, north), abs(south-north), cut, south, north))
    _, _, cut, south, north = min(candidates)
    assigned = (bands[:cut], bands[cut:])
    platforms = {identifier: _plan_scan_tasks(tree, subset)
                 for identifier, subset in zip(platform_ids, assigned)}
    actual = {identifier: plan['length_m'] for identifier, plan in platforms.items()}
    actual_total = sum(actual.values())
    allocation = {
        'method': 'two contiguous south/north task bands by midpoint y',
        'platform_band_order': list(platform_ids),
        'candidate_cut_count': len(cuts), 'selected_cut_index': cut,
        'equal_y_groups_preserved': grouped,
        'same_y_split_required': not grouped,
        'split_y_m': (midpoints_y[cut-1]+midpoints_y[cut])/2,
        'objective': 'min max(scan length + Euclidean inter-task connector estimate); ties by imbalance then cut index',
        'estimated_route_lengths_m': dict(zip(platform_ids, (south, north))),
        'actual_route_lengths_m': actual,
        'actual_length_imbalance_fraction': abs(actual[platform_ids[0]]-actual[platform_ids[1]])/actual_total if actual_total else 0.0,
        'task_count': len(tasks),
        'scan_length_m': sum(task['length_m'] for task in tasks),
        'optimality_claimed': False,
        'launch_and_recovery_included': False,
        'spatial_band_note': 'Bands separate task midpoints; scan footprints and vertical task endpoints may overlap spatially'}
    return {'tasks': tasks, 'platforms': platforms, 'allocation': allocation}
