"""GSHHG 2.3.x native binary polygons, following the distribution README.

The regional reader deliberately supports non-dateline windows in 0..180 E.
Coordinates are geographic degrees; hierarchy levels 1/3 are land, 2/4 water.
"""
import math
import struct
import zipfile


def read_polygons(archive, resolution, bounds):
    if resolution not in ('c', 'l', 'i', 'h', 'f'):
        raise ValueError('Unknown GSHHG resolution')
    west, south, east, north = bounds
    if not all(math.isfinite(v) for v in bounds) or not (0 <= west < east <= 180 and -80 <= south < north <= 80):
        raise ValueError('Expected a non-dateline regional window in 0..180 E')
    polygons = []
    with zipfile.ZipFile(archive) as zipped, zipped.open(f'gshhs_{resolution}.b') as source:
        while header := source.read(44):
            if len(header) != 44:
                raise ValueError('Truncated GSHHG header')
            identifier, n, flag, w, e, s, n_lat, area, area_full, container, ancestor = struct.unpack('>11i', header)
            if n < 3:
                raise ValueError('Invalid shoreline vertex count')
            data = source.read(n * 8)
            if len(data) != n * 8:
                raise ValueError('Truncated GSHHG polygon')
            level = flag & 255
            if level not in (1, 2, 3, 4) or e/1e6 < west or w/1e6 > east or n_lat/1e6 < south or s/1e6 > north:
                continue
            greenwich = (flag >> 16) & 3
            points = [((x-360000000 if greenwich and x > 270000000 else x)/1e6, y/1e6)
                      for x, y in struct.iter_unpack('>ii', data)]
            clipped = clip_polygon(points, bounds)
            if len(clipped) >= 3:
                polygons.append({'id': identifier, 'level': level, 'container': container,
                                 'ancestor': ancestor, 'points_lonlat': clipped,
                                 'source_vertex_count': n})
    return polygons


def clip_polygon(points, bounds):
    """Sutherland-Hodgman polygon clipping to a closed axis-aligned window."""
    west, south, east, north = bounds
    result = list(points)
    if result and result[0] == result[-1]:
        result.pop()
    for axis, limit, sign in ((0, west, 1), (0, east, -1), (1, south, 1), (1, north, -1)):
        previous_points, result = result, []
        if not previous_points:
            break
        previous = previous_points[-1]
        for current in previous_points:
            a, b = sign*(previous[axis]-limit), sign*(current[axis]-limit)
            if (a >= 0) != (b >= 0):
                t = (limit-previous[axis])/(current[axis]-previous[axis])
                crossing = tuple(previous[k]+t*(current[k]-previous[k]) for k in (0, 1))
                result.append(crossing)
            if b >= 0:
                result.append(current)
            previous = current
    return result
