"""Independent sampled coverage and raw closed-cell route audit for A2.

Uses NumPy distances and the preceding independent A1 ray-casting convention.
Does not call the production coverage planner, tree LOS or GEOS footprint code.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from berthing_planner.validation.chart import inside


def point_segment_distance_squared(points, start, end):
    """Euclidean distance to a CLOSED segment, including point segments."""
    a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    delta = b-a
    denominator = float(delta@delta)
    if denominator == 0:
        return np.sum((points-a)**2, axis=1)
    t = np.clip(((points-a)@delta)/denominator, 0, 1)
    return np.sum((points-a-t[:, None]*delta)**2, axis=1)


def active_pass_counts(points, legs, radius):
    """Each active straight pass contributes at most one hit per point.

    Intermediate vertices within one pass cannot create artificial repeats.
    Transit legs never scan, even if an erroneous scanning flag is present;
    route_checks reports such an erroneous flag separately.
    """
    counts = np.zeros(len(points), dtype=np.uint32)
    for leg in legs:
        if not leg['scanning'] or leg['kind'] == 'transit':
            continue
        xy = np.asarray(leg['points'], dtype=float)
        lower, upper = xy.min(axis=0)-radius, xy.max(axis=0)+radius
        indices = np.flatnonzero(np.all((points >= lower) & (points <= upper), axis=1))
        if not len(indices):
            continue
        distances = np.full(len(indices), np.inf)
        pairs = zip(xy, xy[1:]) if len(xy) > 1 else [(xy[0], xy[0])]
        for a, b in pairs:
            distances = np.minimum(distances, point_segment_distance_squared(points[indices], a, b))
        counts[indices[distances <= radius*radius]] += 1
    return counts


def closed_segment_blocked_cells(start, end, blocked, bounds):
    """Independent slab intersection with every raw blocked cell in the ROI.

    Touching an edge/corner is a hit. Rectangles are not merged, dilated or
    sampled at centres. Returned columns are [row, column] in bottom-up order.
    """
    a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    x0, y0, x1, y1 = bounds
    ny, nx = blocked.shape
    dx, dy = (x1-x0)/nx, (y1-y0)/ny
    low, high = np.minimum(a, b), np.maximum(a, b)
    # ceil(min/step)-1 includes the cell immediately below a closed grid edge.
    ilo = max(0, math.ceil((low[0]-x0)/dx)-1)
    ihi = min(nx-1, math.floor((high[0]-x0)/dx))
    jlo = max(0, math.ceil((low[1]-y0)/dy)-1)
    jhi = min(ny-1, math.floor((high[1]-y0)/dy))
    if ilo > ihi or jlo > jhi:
        return np.empty((0, 2), dtype=int)
    rows, columns = np.nonzero(blocked[jlo:jhi+1, ilo:ihi+1])
    rows, columns = rows+jlo, columns+ilo
    lower = np.column_stack((x0+columns*dx, y0+rows*dy))
    upper = lower+np.array([dx, dy])
    enter, leave = np.zeros(len(rows)), np.ones(len(rows))
    intersects = np.ones(len(rows), dtype=bool)
    for axis in (0, 1):
        delta = b[axis]-a[axis]
        if delta == 0:
            intersects &= (lower[:, axis] <= a[axis]) & (a[axis] <= upper[:, axis])
        else:
            t0 = (lower[:, axis]-a[axis])/delta
            t1 = (upper[:, axis]-a[axis])/delta
            enter = np.maximum(enter, np.minimum(t0, t1))
            leave = np.minimum(leave, np.maximum(t0, t1))
    return np.column_stack((rows, columns))[intersects & (enter <= leave)]


def _points(values):
    points = np.asarray(values, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        raise ValueError('Expected a nonempty array of finite XY points')
    return points


def _clean_path(points):
    return points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-7]]


def route_checks(result):
    """Check the declared legs, active-pass straightness and path identity."""
    path = _points(result['path'])
    expected, gaps, length_errors, curved, scanning_transits = [], [], [], [], []
    total = 0.0
    for index, leg in enumerate(result['legs']):
        if leg['kind'] not in ('sweep', 'transit', 'supplement') or not isinstance(leg['scanning'], bool):
            raise ValueError('Unexpected leg kind or non-boolean scanning flag')
        xy = _points(leg['points'])
        length = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
        total += length
        length_errors.append(abs(length-leg['length_m']))
        if expected:
            gaps.append(float(np.linalg.norm(xy[0]-expected[-1])))
            expected.extend(xy[1:] if gaps[-1] <= 1e-7 else xy)
        else:
            expected.extend(xy)
        if leg['kind'] == 'transit' and leg['scanning']:
            scanning_transits.append(index)
        if leg['scanning'] and leg['kind'] != 'transit':
            direction = xy[-1]-xy[0]
            norm2 = float(direction@direction)
            if norm2 == 0:
                straight = np.all(np.linalg.norm(xy-xy[0], axis=1) <= 1e-7)
            else:
                parameter = ((xy-xy[0])@direction)/norm2
                straight = (np.all(point_segment_distance_squared(xy, xy[0], xy[-1]) <= 1e-14)
                            and np.all(np.diff(parameter) >= -1e-9))
            if not straight:
                curved.append(index)
    path_length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    clean = _clean_path(path)
    assembled = _clean_path(np.asarray(expected)) if expected else np.empty((0, 2))
    same = clean.shape == assembled.shape and bool(np.all(np.abs(clean-assembled) <= 1e-7))
    check = {
        'path_matches_ordered_legs': same,
        'maximum_leg_gap_m': max(gaps, default=0),
        'maximum_leg_length_error_m': max(length_errors, default=0),
        'recomputed_path_length_m': path_length,
        'recomputed_leg_length_m': total,
        'path_vs_declared_length_error_m': abs(path_length-result['length_m']),
        'legs_vs_declared_length_error_m': abs(total-result['length_m']),
        'non_straight_active_leg_indices': curved,
        'transit_legs_with_scanning_flag': scanning_transits,
        'active_pass_count': sum(leg['scanning'] and leg['kind'] != 'transit' for leg in result['legs']),
        'transit_leg_count': sum(leg['kind'] == 'transit' for leg in result['legs'])}
    check['passed'] = (same and check['maximum_leg_gap_m'] <= 1e-7
                       and check['maximum_leg_length_error_m'] <= 1e-6
                       and check['path_vs_declared_length_error_m'] <= 1e-6
                       and check['legs_vs_declared_length_error_m'] <= 1e-6
                       and not curved and not scanning_transits)
    return check


def run(*, coverage_dir, prepared_dir, grid_dir, json_out):
    if json_out.exists():
        raise FileExistsError('Preserving the preceding audit; use a new JSON output')
    files = {'result': coverage_dir/'result.json', 'config': coverage_dir/'config.json',
             'target': prepared_dir/'searchable.geojson',
             'a1_manifest': prepared_dir/'manifest.json',
             'a1_summary': prepared_dir/'summary.json', 'grid': grid_dir/'grid.json'}
    data = {name: json.loads(p.read_text()) for name, p in files.items()}
    result, config, source, grid = data['result'], data['config'], data['a1_summary'], data['grid']
    radius = config['swath_width_m']/2
    if not radius > 0 or not 0 < config['target_coverage_ratio'] <= 1:
        raise ValueError('Positive swath and a target ratio in (0,1] are required')
    route = route_checks(result)
    declared = result['vector_coverage']
    digest = hashlib.sha256(files['target'].read_bytes()).hexdigest()
    reported_digest = result.get('target_geometry_sha256', declared.get('target_geometry_sha256'))
    immutable = digest == data['a1_manifest']['derived_files_sha256']['searchable.geojson'] == reported_digest
    target_area_matches = abs(declared['target_area_m2']-source['searchable_area_m2']) <= 1e-6
    shape = data['target']['features'][0]['geometry']
    xmin, ymin, xmax, ymax = source['bbox_local_m']
    step = 10.0
    xx, yy = np.meshgrid(np.arange(xmin+step/2, xmax, step), np.arange(ymin+step/2, ymax, step))
    all_points = np.column_stack((xx.ravel(), yy.ravel()))
    contained, boundary = inside(shape, all_points)
    target_points = all_points[contained]
    target_boundary = boundary[contained]
    counts = active_pass_counts(target_points, result['legs'], radius)
    interior = counts[~target_boundary]
    denominator = len(interior)
    if not denominator:
        raise ValueError('No strict-interior target sample points')
    covered, repeated, missed = int(np.sum(interior >= 1)), int(np.sum(interior >= 2)), int(np.sum(interior == 0))
    sampled = {'target_area_m2': denominator*step*step,
               'covered_area_m2': covered*step*step, 'repeated_area_m2': repeated*step*step,
               'missed_area_m2': missed*step*step, 'coverage_ratio': covered/denominator,
               'repeated_ratio': repeated/denominator, 'missed_ratio': missed/denominator}
    comparison = {key: sampled[key]-declared[key] for key in sampled}
    blocked = np.asarray(grid['rows_bottom_to_top'], dtype=bool)
    if blocked.shape != (grid['n'], grid['n']):
        raise ValueError('Grid shape differs from its declared n')
    path = np.asarray(result['path'], dtype=float)
    bounds = grid['bounds']
    outside = int(np.sum((path[:, 0] < bounds[0]) | (path[:, 0] > bounds[2]) |
                         (path[:, 1] < bounds[1]) | (path[:, 1] > bounds[3])))
    hits, hit_segments, examples = set(), 0, []
    pairs = zip(path, path[1:]) if len(path) > 1 else [(path[0], path[0])]
    for index, (a, b) in enumerate(pairs):
        cells = closed_segment_blocked_cells(a, b, blocked, bounds)
        if len(cells):
            hit_segments += 1
            hits.update((int(j), int(i)) for j, i in cells)
            if len(examples) < 20:
                examples.append({'segment_index': index, 'blocked_cells_row_col': cells[:20].tolist(),
                                 'total_blocked_cells_hit': len(cells)})
    exact_partition = abs(declared['covered_area_m2']+declared['missed_area_m2']-declared['target_area_m2']) <= max(
        1e-6, declared['target_area_m2']*1e-9)
    exact_order = (0 <= declared['repeated_area_m2'] <= declared['covered_area_m2']+1e-6
                   <= declared['target_area_m2']+2e-6)
    exact_ratios = all(abs(declared[ratio]-declared[area]/declared['target_area_m2']) <= 1e-12
                       for ratio, area in (('coverage_ratio', 'covered_area_m2'),
                                           ('missed_ratio', 'missed_area_m2'),
                                           ('repeated_ratio', 'repeated_area_m2')))
    unique, histogram = np.unique(interior, return_counts=True)
    audit = {
        'method': 'Independent 10 m candidate grid-centre ray casting and NumPy distance to closed active scan segments; raw unmerged blocked-cell slab intersections',
        'sensor_model': 'Ideal unobstructed circular footprint, radius swath_width_m/2; active scan legs only',
        'swath_width_m': config['swath_width_m'], 'radius_m': radius,
        'lane_spacing_m': config['lane_spacing_m'], 'lane_offset_m': config['lane_offset_m'],
        'target_coverage_ratio': config['target_coverage_ratio'],
        'sampling_step_m': step, 'sampling_origin_m': [xmin+step/2, ymin+step/2],
        'sample_grid_centres': len(all_points), 'candidate_closed_grid_centres': len(target_points),
        'candidate_boundary_grid_centres_separately_reported': int(target_boundary.sum()),
        'candidate_interior_grid_centres': denominator,
        'boundary_policy': 'A1 ray casting: 1e-7 m boundary tolerance; target boundary sample points reported separately and excluded from primary sampled ratios; native vector area unchanged',
        'closed_candidate_sample_area_m2': len(target_points)*step*step,
        'closed_candidate_sample_minus_vector_area_m2': len(target_points)*step*step-declared['target_area_m2'],
        'boundary_sample_hit_histogram': {str(int(k)): int(v) for k, v in zip(*np.unique(counts[target_boundary], return_counts=True))},
        'interior_active_pass_hit_histogram': {str(int(k)): int(v) for k, v in zip(unique, histogram)},
        'sampled_interior_metrics': sampled, 'native_vector_metrics': declared,
        'sampled_minus_native_metrics': comparison,
        'target_geometry_sha256': digest, 'target_unchanged': immutable,
        'native_target_area_matches_A1': target_area_matches,
        'native_area_partition_consistent': exact_partition,
        'native_area_order_consistent': exact_order,
        'native_ratios_consistent': exact_ratios,
        'route': route,
        'raw_grid_collision': {'path_points_outside_grid_bounds': outside,
                               'segments_touching_or_crossing_blocked_cells': hit_segments,
                               'unique_blocked_cells_hit': len(hits), 'examples': examples},
        'sampled_coverage_meets_target': sampled['coverage_ratio'] >= config['target_coverage_ratio'],
        'native_coverage_meets_target': declared['coverage_ratio'] >= config['target_coverage_ratio'],
        'input_sha256': {name: hashlib.sha256(p.read_bytes()).hexdigest() for name, p in files.items()},
        'audit_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'ray_casting_script_sha256': hashlib.sha256(Path(__file__).with_name('chart.py').read_bytes()).hexdigest(),
        'limits': 'Finite 10 m sampling is not exact area or a proof that arbitrarily narrow missed slivers are absent. Native area differences are diagnostics, not zero-error expectations. No occlusion, 3D, sensing physics, curvature, speed, controller, hull or operational validation.'}
    audit['passed'] = (immutable and target_area_matches and exact_partition and exact_order and exact_ratios and route['passed']
                       and outside == 0 and hit_segments == 0 and audit['sampled_coverage_meets_target']
                       and audit['native_coverage_meets_target'])
    json_out.write_text(json.dumps(audit, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    if not audit['passed']:
        raise SystemExit(1)
