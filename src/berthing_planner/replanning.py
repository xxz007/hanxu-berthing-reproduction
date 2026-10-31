"""Replace only the obstructed portion of a saved polyline on a finer map.

Fixed-distance sampling chooses splice anchors; exact LOS checks cover every
sample interval, so obstacles between samples are not skipped. The unaffected
original waypoint prefix/suffix remains unchanged apart from inserted anchors.
This is an explicit reconstruction of the local-replanning concept, not author
code. It imposes no curvature or endpoint-heading constraint on the polyline.
"""
import math

from .search import astar


def _point_at(path, lengths, distance):
    travelled = 0.0
    for i, length in enumerate(lengths):
        if length and distance <= travelled + length:
            fraction = max(0.0, (distance - travelled) / length)
            if fraction == 0:
                return path[i], i
            if fraction == 1:
                return path[i + 1], i
            return tuple(path[i][j] + fraction * (path[i + 1][j] - path[i][j])
                         for j in range(2)), i
        travelled += length
    return path[-1], len(path) - 2


def replan_path(tree, path, *, sample_step_m=100.0, anchor_padding_m=0.0,
                parent_policy='theta_relaxation'):
    """Return the fine-map checked path and explicit local-splice evidence.

All path vertices must lie inside this map. If fine-map start/end are blocked,
or A* cannot connect the splice anchors, raise ValueError without moving the
endpoints or relaxing land. Multiple collision spans are covered by one splice
from the first blocked sample interval to the last.
"""
    if not math.isfinite(sample_step_m) or sample_step_m <= 0:
        raise ValueError('Sample step must be finite and positive')
    if not math.isfinite(anchor_padding_m) or anchor_padding_m < 0:
        raise ValueError('Anchor padding must be finite and nonnegative')
    points = [tuple(point) for point in path]
    if len(points) < 2 or any(len(point) != 2 for point in points):
        raise ValueError('A polyline needs at least two 2D points')
    for point in points:
        tree.locate(point)
    if not all(tree.line_of_sight(point, point) for point in (points[0], points[-1])):
        raise ValueError('Original endpoints are blocked or touch a blocked boundary')
    lengths = [math.dist(a, b) for a, b in zip(points, points[1:])]
    first_bad = last_bad = None
    bad_segments = []
    travelled = 0.0
    for i, (a, b, length) in enumerate(zip(points, points[1:], lengths)):
        if not tree.line_of_sight(a, b):
            bad_segments.append(i)
            count = max(1, math.ceil(length / sample_step_m))
            for k in range(count):
                p = a if k == 0 else tuple(a[j] + (b[j] - a[j]) * k / count for j in range(2))
                q = b if k + 1 == count else tuple(
                    a[j] + (b[j] - a[j]) * (k + 1) / count for j in range(2))
                if not tree.line_of_sight(p, q):
                    if first_bad is None:
                        first_bad = travelled + length * k / count
                    last_bad = travelled + length * (k + 1) / count
        travelled += length
    if first_bad is None:
        return {'path': points, 'changed': False, 'length_m': travelled,
                'original_length_m': travelled, 'blocked_segment_indices': [],
                'preserved_prefix': points, 'preserved_suffix': [],
                'replacement_start': None, 'replacement_end': None,
                'replacement_search': None}

    start_distance = max(0.0, first_bad - anchor_padding_m)
    end_distance = min(travelled, last_bad + anchor_padding_m)
    start, first_index = _point_at(points, lengths, start_distance)
    end, last_index = _point_at(points, lengths, end_distance)
    prefix = points[:first_index + 1]
    if prefix[-1] != start:
        prefix.append(start)
    suffix = points[last_index + 1:]
    if not suffix or suffix[0] != end:
        suffix.insert(0, end)
    replacement = astar(tree, start, end, parent_policy=parent_policy)
    final = prefix[:-1] + replacement['path'] + suffix[1:]
    if not all(tree.line_of_sight(a, b) for a, b in zip(final, final[1:])):
        raise ValueError('Spliced path does not pass the fine-map geometry check')
    return {'path': final, 'changed': True,
            'length_m': sum(math.dist(a, b) for a, b in zip(final, final[1:])),
            'original_length_m': travelled, 'blocked_segment_indices': bad_segments,
            'preserved_prefix': prefix, 'preserved_suffix': suffix,
            'replacement_start': start, 'replacement_end': end,
            'replacement_search': replacement,
            'anchor_distances_m': [start_distance, end_distance],
            'replaced_original_length_m': end_distance - start_distance,
            'prefix_length_m': start_distance, 'suffix_length_m': travelled - end_distance}
