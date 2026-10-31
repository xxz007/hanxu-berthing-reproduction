"""Build conservative coarse/fine GSHHG land grids for the R8 experiment.

Existing research environment only: NumPy and SciPy. The planner remains stdlib.
Land is filled by GSHHG hierarchy, then closed shoreline cells are blocked.
The 50m buffer expands the resulting closed land-cell rectangles, never clears land.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from berthing_planner.coordinates import Mercator
from berthing_planner.gshhg import read_polygons
import numpy as np
from scipy.ndimage import binary_dilation


def rasterize(polygons, bounds, n, margin_m):
    """Pixel-center hierarchy fill plus exact closed-cell shoreline supercover."""
    x0, y0, x1, y1 = bounds
    if n < 2 or n & (n-1) or not x0 < x1 or not y0 < y1 or not 0 <= margin_m:
        raise ValueError('Expected ordered bounds, power-of-two grid and nonnegative margin')
    dx, dy = (x1-x0)/n, (y1-y0)/n
    land = np.zeros((n, n), dtype=bool)
    for polygon in sorted(polygons, key=lambda p: p['level']):
        points = np.asarray(polygon['points_EN_m'], dtype=float)
        before, after = points, np.roll(points, -1, axis=0)
        for j in range(max(0, math.floor((points[:, 1].min()-y0)/dy)),
                       min(n, math.ceil((points[:, 1].max()-y0)/dy))):
            y = y0+(j+.5)*dy
            active = (before[:, 1] > y) != (after[:, 1] > y)
            a, b = before[active], after[active]
            crossings = np.sort(a[:, 0]+(y-a[:, 1])*(b[:, 0]-a[:, 0])/(b[:, 1]-a[:, 1]))
            if len(crossings) % 2:
                raise ValueError('Odd scanline crossing count in source polygon')
            for left, right in zip(crossings[::2], crossings[1::2]):
                i0 = max(0, math.ceil((left-x0)/dx-.5))
                i1 = min(n, math.floor((right-x0)/dx-.5)+1)
                land[j, i0:i1] = polygon['level'] % 2 == 1
    # Every cell touched by a boundary is blocked, including both sides of grid lines.
    for polygon in polygons:
        points = [((x-x0)/dx, (y-y0)/dy) for x, y in polygon['points_EN_m']]
        for a, b in zip(points, points[1:]+points[:1]):
            cuts = {0.0, 1.0}
            for axis in (0, 1):
                delta = b[axis]-a[axis]
                if delta:
                    lo, hi = sorted((a[axis], b[axis]))
                    cuts.update((k-a[axis])/delta for k in range(math.ceil(lo), math.floor(hi)+1)
                                if 0 < (k-a[axis])/delta < 1)
            cuts = sorted(cuts)
            for t in cuts+[(u+v)/2 for u, v in zip(cuts, cuts[1:])]:
                u, v = (a[k]+t*(b[k]-a[k]) for k in (0, 1))
                cols = (round(u)-1, round(u)) if abs(u-round(u)) < 1e-8 else (math.floor(u),)
                rows = (round(v)-1, round(v)) if abs(v-round(v)) < 1e-8 else (math.floor(v),)
                for j in rows:
                    for i in cols:
                        if 0 <= i < n and 0 <= j < n:
                            land[j, i] = True
    if margin_m == 0:
        return land, land.copy()
    rx, ry = math.ceil(margin_m/dx)+1, math.ceil(margin_m/dy)+1
    offsets = np.zeros((2*ry+1, 2*rx+1), dtype=bool)
    for j in range(-ry, ry+1):
        for i in range(-rx, rx+1):
            offsets[j+ry, i+rx] = math.hypot(max(abs(i)-1, 0)*dx, max(abs(j)-1, 0)*dy) <= margin_m
    return land, binary_dilation(land, structure=offsets)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/singapore_r8.json')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    archive = ROOT/config['source']['archive']
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    if digest != config['source']['sha256']:
        raise ValueError('GSHHG archive differs from the pinned official release')
    output = args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_gshhg_r8_maps')
    output.mkdir(parents=True, exist_ok=False)
    projection = Mercator(**config['projection'])
    summary = {}
    for name in ('coarse', 'fine'):
        settings = config[name]; bounds = settings['bounds_EN_m']; n = settings['grid_n']
        west, south = projection.inverse(*bounds[:2])
        east, north = projection.inverse(*bounds[2:])
        source = read_polygons(archive, settings['resolution'], (west, south, east, north))
        polygons = [{**{k: v for k, v in p.items() if k != 'points_lonlat'},
                     'points_EN_m': [projection.forward(*point) for point in p['points_lonlat']]}
                    for p in source]
        land, buffered = rasterize(polygons, bounds, n, config['safety_margin_m'])
        data = {'schema_version': 1, 'status': 'GSHHG_independent_land_grid_not_author_map',
                'bounds': bounds, 'n': n, 'rows_bottom_to_top': buffered.astype(np.uint8).tolist(),
                'land_rows_bottom_to_top': land.astype(np.uint8).tolist(),
                'source_resolution': settings['resolution'], 'margin_m': config['safety_margin_m'],
                'rasterization': 'hierarchical interior fill + closed shoreline supercover; closed-cell conservative margin',
                'geographic_bounds_lonlat': [west, south, east, north], 'source_sha256': digest}
        (output/(name+'_map.json')).write_text(json.dumps(data, separators=(',', ':'))+'\n')
        (output/(name+'_polygons.json')).write_text(json.dumps(polygons, separators=(',', ':'))+'\n')
        summary[name] = {'polygons': len(polygons), 'vertices': sum(len(p['points_EN_m']) for p in polygons),
                         'grid_n': n, 'cell_dimensions_m': [(bounds[2]-bounds[0])/n, (bounds[3]-bounds[1])/n],
                         'land_cells': int(land.sum()), 'buffered_cells': int(buffered.sum()),
                         'land_never_cleared': bool(np.all(buffered[land]))}
        print(name, summary[name], flush=True)
    shutil.copy2(args.config, output/'config.json')
    snapshot = output/'code_snapshot'; snapshot.mkdir()
    for path in (Path(__file__), ROOT/'src/berthing_planner/gshhg.py', ROOT/'src/berthing_planner/coordinates.py'):
        shutil.copy2(path, snapshot/path.name)
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    (output/'provenance.json').write_text(json.dumps({
        'retrieved_source': config['source'], 'generated_utc': datetime.now(timezone.utc).isoformat(),
        'input_sha256': {args.config.name: hashlib.sha256(args.config.read_bytes()).hexdigest(), archive.name: digest},
        'output_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.glob('*.json')},
        'numpy': np.__version__, 'conditions': config['evidence']}, ensure_ascii=False, indent=2)+'\n')
    print(output)


if __name__ == '__main__':
    main()
