"""Explicit engineering merge of adjacent turns, separate from source G2CBS.

The 2008 reference supplies single-corner bisection, not this merging rule.
Original points and every geometric substitution are retained in the result.
"""
import math

from .curves import collision_free, curvature_maximum, g2_smooth, wrap
from .g2_bisection import bisect_corners

_COEFFICIENT = (2*(math.sqrt(6)-1)/5+4)*(7.2364+6)/54


def _budget(path, maximum_curvature, parts):
    lengths = [math.dist(a, b) for a, b in zip(path, path[1:])]
    trims = [0.0]*len(path)
    turns = []
    for i, (a, b, c) in enumerate(zip(path, path[1:], path[2:]), 1):
        u = tuple((b[j]-a[j])/lengths[i-1] for j in (0, 1))
        v = tuple((c[j]-b[j])/lengths[i] for j in (0, 1))
        gamma = math.atan2(u[0]*v[1]-u[1]*v[0], sum(u[j]*v[j] for j in (0, 1)))
        if abs(gamma) > math.pi-1e-6:
            raise ValueError('Cannot construct an inscribed turn at a reversal')
        if abs(gamma) >= 1e-7:
            denominator = 1 if parts is None else math.cos(gamma/(2*parts))
            trims[i] = _COEFFICIENT*math.tan(abs(gamma)/2)/(maximum_curvature*denominator)
        turns.append({'node_index': i, 'signed_turn_rad': gamma, 'required_trim_m': trims[i]})
    segments = [{'segment_index': i, 'length_m': length,
                 'required_trim_m': trims[i]+trims[i+1],
                 'remaining_m': length-trims[i]-trims[i+1]} for i, length in enumerate(lengths)]
    return {'path': path, 'parts_per_corner': parts, 'corners': turns, 'segments': segments,
            'insufficient_segments': [s['segment_index'] for s in segments if s['remaining_m'] < -1e-8]}


def subdivision_budget(path, maximum_curvature, parts=1):
    """Budget for uniform equal-angle subdivision; ``None`` gives its limit.

    Derivation from 2008 Fig.8: d_n = (C/k)*tan(|gamma|/2)/cos(gamma/(2n)).
    n=1 is the basic pair, n=2 is the published bisection, and n tending to
    infinity is a lower bound ONLY for this independent-corner construction.
    This function does not construct curves for n>2 or prove global infeasibility.
    """
    if not math.isfinite(maximum_curvature) or maximum_curvature <= 0:
        raise ValueError('Maximum curvature must be finite and positive')
    if parts is not None and (not isinstance(parts, int) or isinstance(parts, bool) or parts < 1):
        raise ValueError('Subdivision count must be a positive integer or None')
    if len(path) < 2 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in path):
        raise ValueError('Path must contain at least two finite 2D points')
    path = [tuple(p) for p in path]
    if any(math.dist(a, b) <= 1e-8 for a, b in zip(path, path[1:])):
        raise ValueError('Consecutive points must be distinct')
    return _budget(path, maximum_curvature, parts)


def merge_adjacent_turns(path, maximum_curvature, obstacles, bounds):
    """Try complete smoothing, then merge budget-conflicting same-sign corners.

    Engineering rule: replace a conflicting adjacent pair with the intersection
    of its two exterior tangent rays. Then try the basic construction and the
    2008 bisection again. No LOS shortcut, endpoint movement, RS replacement,
    obstacle modification or claim of author equivalence is involved. Greedy
    selection is deterministic and is not a complete path-planning algorithm.
    """
    initial = subdivision_budget(path, maximum_curvature)
    original, current = initial['path'], initial['path'][:]
    groups = [[i] for i in range(len(current))]
    merges, attempts, rejected = [], [], []
    original_headings = [math.atan2(b[1]-a[1], b[0]-a[0])
                         for a, b in ((original[0], original[1]), (original[-2], original[-1]))]
    while True:
        basic, bisected = _budget(current, maximum_curvature, 1), _budget(current, maximum_curvature, 2)
        for method, budget in (('basic', basic), ('bisection_2008', bisected)):
            if budget['insufficient_segments']:
                attempts.append({'method': method, 'point_count': len(current),
                                 'status': 'insufficient_budget', 'budget': budget})
                continue
            pieces = (g2_smooth(current, maximum_curvature) if method == 'basic'
                      else bisect_corners(current, maximum_curvature)['pieces'])
            joins = [(a.evaluate(1), b.evaluate(0)) for a, b in zip(pieces, pieces[1:])]
            errors = {
                'position_m': max((math.dist(a[:2], b[:2]) for a, b in joins), default=0),
                'heading_rad': max((abs(wrap(a[2]-b[2])) for a, b in joins), default=0),
                'curvature_per_m': max((abs(a[3]-b[3]) for a, b in joins), default=0)}
            endpoints = [pieces[0].evaluate(0), pieces[-1].evaluate(1)]
            endpoint_error = max(math.dist(endpoints[0][:2], original[0]),
                                 math.dist(endpoints[1][:2], original[-1]))
            endpoint_heading_error = max(abs(wrap(p[2]-h)) for p, h in zip(endpoints, original_headings))
            maximum = max(map(curvature_maximum, pieces), default=0)
            clear = collision_free(pieces, obstacles, bounds)
            passed = (clear and maximum <= maximum_curvature+1e-10 and endpoint_error < 1e-6
                      and endpoint_heading_error < 1e-8 and errors['position_m'] < 1e-6
                      and errors['heading_rad'] < 1e-8 and errors['curvature_per_m'] < 1e-10)
            record = {'method': method, 'point_count': len(current), 'collision_free': clear,
                      'maximum_actual_curvature_per_m': maximum, 'maximum_join_errors': errors,
                      'endpoint_position_error_m': endpoint_error,
                      'endpoint_heading_error_rad': endpoint_heading_error,
                      'status': 'engineering_geometry_passed' if passed else 'geometry_rejected'}
            attempts.append(record)
            if passed:
                return {'status': 'engineering_geometry_passed', 'author_equivalence': False,
                        'original_path': original, 'merged_virtual_path': current,
                        'original_index_groups': groups, 'merges': merges, 'attempts': attempts,
                        'pieces': pieces, 'selected_method': method, 'checks': record}
            rejected = pieces
        eligible = []
        turns = {c['node_index']: c['signed_turn_rad'] for c in bisected['corners']}
        for i in bisected['insufficient_segments']:
            if i == 0 or i+2 >= len(current) or turns[i]*turns[i+1] <= 0:
                continue
            if abs(turns[i]+turns[i+1]) > math.pi-1e-6:
                continue
            a, b, c, d = current[i-1:i+3]
            u = tuple((b[j]-a[j])/math.dist(a, b) for j in (0, 1))
            v = tuple((d[j]-c[j])/math.dist(c, d) for j in (0, 1))
            det = u[0]*v[1]-u[1]*v[0]
            if abs(det) < 1e-10:
                continue
            delta = (c[0]-a[0], c[1]-a[1])
            t = (delta[0]*v[1]-delta[1]*v[0])/det
            outgoing_t = (delta[0]*u[1]-delta[1]*u[0])/det
            if t <= 1e-8 or outgoing_t >= math.dist(c, d)-1e-8:
                continue
            vertex = tuple(a[j]+t*u[j] for j in (0, 1))
            eligible.append((i, vertex))
        if not eligible:
            return {'status': 'no_valid_engineering_merge', 'author_equivalence': False,
                    'original_path': original, 'merged_virtual_path': current,
                    'original_index_groups': groups, 'merges': merges, 'attempts': attempts,
                    'pieces': [], 'rejected_pieces': rejected}
        i, vertex = eligible[0]
        merges.append({'original_indices': groups[i]+groups[i+1],
                       'replaced_vertices': current[i:i+2], 'virtual_vertex': vertex,
                       'rule': 'intersection_of_exterior_tangent_rays'})
        current = current[:i]+[vertex]+current[i+2:]
        groups = groups[:i]+[groups[i]+groups[i+1]]+groups[i+2:]
