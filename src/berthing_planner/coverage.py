"""Single-platform horizontal sweep legs with safe, scan-off A* connections.

This is a geometric coverage route, not a curvature-constrained or timed vessel
trajectory. Cell-centre sweep positions are reported rather than silently
equating the requested lane phase with the actual raster row.
"""
from functools import lru_cache
import math
from weakref import WeakKeyDictionary, ref

from .search import astar, shortcut


class _CachedTree:
    """Read-only query proxy; the base occupancy must stay unchanged."""
    def __init__(self, tree):
        self._base = ref(tree)

    def __getattr__(self, name):
        return getattr(self._base(), name)

    @lru_cache(maxsize=65536)
    def locate(self, point):
        return self._base().locate(point)

    @lru_cache(maxsize=65536)
    def neighbors(self, code):
        return self._base().neighbors(code)

    def line_of_sight(self, start, end):
        a, b = tuple(start), tuple(end)
        return self._visible(a, b) if a <= b else self._visible(b, a)

    @lru_cache(maxsize=65536)
    def _visible(self, a, b):
        return self._base().line_of_sight(a, b)


_QUERIES = WeakKeyDictionary()


def _cached_tree(tree):
    if tree not in _QUERIES:
        _QUERIES[tree] = _CachedTree(tree)
    return _QUERIES[tree]


def append_sweep(tree, plan, start, end, kind='supplement'):
    """Atomically append a safe scanning line and any scan-off connection.

    ``plan`` is updated in place and returned. On a blocked scan or unavailable
    connector it is unchanged and a ValueError is raised. A one-cell free run
    retains its zero-length scanning leg; this does not assign dwell time.
    """
    if kind not in ('sweep', 'supplement'):
        raise ValueError('A scanning leg must be sweep or supplement')
    a, b = tuple(start), tuple(end)
    if len(a) != 2 or len(b) != 2 or not all(math.isfinite(v) for v in (*a, *b)):
        raise ValueError('Scan endpoints must be finite 2D coordinates')
    queries = _cached_tree(tree)
    if not queries.line_of_sight(a, b):
        raise ValueError('Scanning leg crosses or touches a blocked cell')
    added = []
    if plan['path']:
        previous = tuple(plan['path'][-1])
        if previous != a:
            direct = queries.line_of_sight(previous, a)
            connection = [previous, a] if direct else shortcut(
                queries, astar(queries, previous, a, parent_policy='theta_relaxation')['path'])
            if not all(queries.line_of_sight(p, q) for p, q in zip(connection, connection[1:])):
                raise ValueError('Connection failed the occupancy geometry check')
            added.append({'kind': 'transit', 'scanning': False,
                          'points': [list(p) for p in connection],
                          'length_m': sum(math.dist(p, q) for p, q in zip(connection, connection[1:])),
                          'connection_method': 'direct_LOS' if direct else 'theta_astar_then_LOS_shortcut'})
    added.append({'kind': kind, 'scanning': True, 'points': [list(a), list(b)],
                  'length_m': math.dist(a, b)})
    for leg in added:
        for point in leg['points']:
            if not plan['path'] or point != plan['path'][-1]:
                plan['path'].append(point)
        plan['legs'].append(leg)
    plan['length_m'] = sum(leg['length_m'] for leg in plan['legs'])
    plan['scan_length_m'] = sum(leg['length_m'] for leg in plan['legs'] if leg['scanning'])
    plan['transit_length_m'] = sum(leg['length_m'] for leg in plan['legs'] if not leg['scanning'])
    plan['supplement_length_m'] = sum(leg['length_m'] for leg in plan['legs'] if leg['kind'] == 'supplement')
    plan['scan_leg_count'] = sum(leg['scanning'] for leg in plan['legs'])
    headings = [math.atan2(q[1]-p[1], q[0]-p[0])
                for p, q in zip(plan['path'], plan['path'][1:]) if p != q]
    plan['direction_change_count'] = sum(
        abs(math.remainder(after-before, 2*math.pi)) > 1e-9
        for before, after in zip(headings, headings[1:]))
    return plan


def plan_coverage(tree, rows, *, lane_spacing_m, lane_offset_m):
    """Sweep every free run on each selected row, in alternating row directions.

    rows must be the same bottom-up 0/1 occupancy used to construct tree. Lane
    phase is measured from bounds[1]. Each requested y uses its nearest raster
    row centre, breaking midpoint ties toward the larger row index. Spacing
    below one cell height is rejected rather than merging requested lanes.
    Any disconnected run raises; no obstacle or requested run is skipped.
    """
    x0, y0, x1, y1 = tree.bounds
    n = tree.n
    if len(rows) != n or any(len(row) != n for row in rows):
        raise ValueError('Sweep occupancy dimensions must match the planning tree')
    dx, dy = (x1-x0)/n, (y1-y0)/n
    if not all(math.isfinite(v) for v in (lane_spacing_m, lane_offset_m)):
        raise ValueError('Lane spacing and phase must be finite')
    if lane_spacing_m < dy:
        raise ValueError('Lane spacing must be at least one cell height')
    if not 0 <= lane_offset_m < lane_spacing_m:
        raise ValueError('Lane phase must lie in [0, lane_spacing_m)')
    plan = {'path': [], 'legs': [], 'length_m': 0.0,
            'lane_spacing_m': lane_spacing_m, 'lane_offset_m': lane_offset_m,
            'row_snap_rule': 'nearest cell centre; midpoint ties choose higher row',
            'scan_rows': [],
            'direction_change_definition': 'heading changes between nonzero polyline segments, including transit; not propulsion reversals'}
    lane = 0
    while (requested_y := y0+lane_offset_m+lane*lane_spacing_m) < y1:
        row_index = math.floor((requested_y-y0)/dy)
        actual_y = y0+(row_index+.5)*dy
        runs = []
        first = None
        for col, blocked in enumerate(rows[row_index]):
            if not blocked and first is None:
                first = col
            if first is not None and (blocked or col == n-1):
                last = col-1 if blocked else col
                runs.append((first, last))
                first = None
        direction = 1 if lane % 2 == 0 else -1
        plan['scan_rows'].append({'requested_y_m': requested_y, 'row_index': row_index,
                                  'actual_y_m': actual_y, 'direction': direction,
                                  'free_run_count': len(runs)})
        for first, last in runs[::direction]:
            a, b = ((first, last) if direction == 1 else (last, first))
            append_sweep(tree, plan, (x0+(a+.5)*dx, actual_y),
                         (x0+(b+.5)*dx, actual_y), kind='sweep')
        lane += 1
    if not plan['legs']:
        raise ValueError('No free scanning run on any requested lane')
    plan['actual_scan_rows'] = [row['row_index'] for row in plan['scan_rows'] if row['free_run_count']]
    return plan
