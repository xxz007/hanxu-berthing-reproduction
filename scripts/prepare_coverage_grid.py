"""Conservative closed-cell planning grid for a fixed A1 candidate polygon.

The original polygon and its holes remain unchanged. A declared numerical
inset is used only for the planning mask, followed by GDAL ALL_TOUCHED burning
of its complement and an exhaustive native GEOS whole-cell inclusion check.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

from chart_geometry import Geos

ROOT = Path(__file__).resolve().parents[1]
BOUNDS = (0.0, 0.0, 5120.0, 5120.0)
GRID_N = 1024
NUMERICAL_INSET_M = 1e-3
SOURCES = {
    'GDAL_ALL_TOUCHED': 'https://gdal.org/en/stable/programs/gdal_rasterize.html#cmdoption-gdal_rasterize-at',
    'GEOS_C_API': 'https://libgeos.org/doxygen/geos__c_8h.html'}


def rasterize_closed_cell_grid(geometry, crs, bounds, n, output_dir, *,
                               gdal_bin=None, geos_library=None,
                               numerical_inset_m=NUMERICAL_INSET_M):
    import numpy as np
    from scipy.ndimage import find_objects, label

    bounds = tuple(float(v) for v in bounds)
    if (len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
            or bounds[2] <= bounds[0] or bounds[3] <= bounds[1]):
        raise ValueError('Grid bounds must have positive finite area')
    if type(n) is not int or n < 1 or n & (n - 1):
        raise ValueError('Grid size must be a positive power of two')
    if not math.isfinite(numerical_inset_m) or numerical_inset_m <= 0:
        raise ValueError('A declared positive numerical inset is required')
    if geometry['type'] not in ('Polygon', 'MultiPolygon'):
        raise ValueError('The fixed candidate must be a polygon or multipolygon')
    if not crs or crs.get('type') != 'name' or not crs.get('properties', {}).get('name'):
        raise ValueError('An explicit projected source CRS is required')
    if gdal_bin is None:
        found = shutil.which('gdal_rasterize')
        gdal_bin = Path(found).parent if found else Path('/Applications/Postgres.app/Contents/Versions/18/bin')
    gdal_bin = Path(gdal_bin)
    if not all((gdal_bin / tool).is_file() for tool in ('gdal_rasterize', 'gdal_translate', 'gdalinfo')):
        raise ValueError('Existing GDAL CLI required; no package is installed automatically')
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    commands = []

    def execute(command):
        result = subprocess.run([str(v) for v in command], capture_output=True, text=True)
        commands.append({'argv': [str(v) for v in command], 'returncode': result.returncode,
                         'stdout': result.stdout, 'stderr': result.stderr})
        (output_dir / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        if result.returncode:
            raise RuntimeError(result.stderr[:1500] or result.stdout[:1500])
        return result.stdout

    x0, y0, x1, y1 = bounds
    dx, dy = (x1 - x0) / n, (y1 - y0) / n
    box = {'type': 'Polygon', 'coordinates': [[[x0, y0], [x1, y0],
                                             [x1, y1], [x0, y1], [x0, y0]]]}
    with Geos(geos_library) as geos:
        source_measure = geos.measure(geometry)
        if not source_measure['valid']:
            raise ValueError('Invalid fixed candidate; no geometry repair was applied')
        if source_measure['area_m2'] and not geos.covers(box, geometry):
            raise ValueError('Grid bounds do not cover the entire fixed candidate')
        inset = geos.operation('buffer', geometry, distance=-numerical_inset_m)
        inset_measure = geos.measure(inset)
        if inset_measure['area_m2'] and not geos.covers(geometry, inset):
            raise ValueError('Numerical inset extends outside the fixed candidate')
        blocked = geos.operation('difference', box, inset)
        feature = {'type': 'FeatureCollection', 'name': 'blocking_complement', 'crs': crs,
                   'features': [{'type': 'Feature', 'properties': {}, 'geometry': blocked}]}
        complement = output_dir / 'blocking_complement.geojson'
        complement.write_text(json.dumps(feature, indent=2) + '\n')
        version = execute([gdal_bin / 'gdal_rasterize', '--version']).strip()
        raster = output_dir / 'blocked.tif'
        execute([gdal_bin / 'gdal_rasterize', '-q', '-at', '-burn', '1', '-init', '0',
                 '-ot', 'Byte', '-of', 'GTiff', '-te', *map(str, bounds), '-ts', str(n), str(n),
                 '-a_srs', crs['properties']['name'], complement, raster])
        info = json.loads(execute([gdal_bin / 'gdalinfo', '-json', raster]))
        expected_transform = [x0, dx, 0.0, y1, 0.0, -dy]
        if info['size'] != [n, n] or info['geoTransform'] != expected_transform:
            raise ValueError('GDAL dimensions or north-down affine differ from the declared grid')
        # Rasterize in GeoTIFF so the complete source CRS is retained. ENVI can
        # round-trip a projected CRS differently; use it only for raw-byte export.
        raw = output_dir / 'blocked.bin'
        execute([gdal_bin / 'gdal_translate', '-q', '-of', 'ENVI', raster, raw])
        if info['bands'][0]['type'] != 'Byte' or raw.stat().st_size != n*n:
            raise ValueError('Expected one unpacked Byte per cell')
        top_down = np.fromfile(raw, dtype=np.uint8).reshape(n, n)
        if not np.isin(top_down, (0, 1)).all():
            raise ValueError('Raster contains values other than 0 free / 1 blocked')
        bottom_up = np.flipud(top_down).copy()
        if not np.array_equal(np.flipud(bottom_up), top_down):
            raise ValueError('Raster row-direction conversion failed')
        indices = np.argwhere(bottom_up == 0)

        def rectangles():
            return ((x0 + int(i)*dx, y0 + int(j)*dy,
                     x0 + (int(i)+1)*dx, y0 + (int(j)+1)*dy) for j, i in indices)

        original_covers = geos.covers_rectangles(geometry, rectangles())
        inset_covers = geos.covers_rectangles(inset, rectangles())
        failed_original = [indices[k].tolist() for k, ok in enumerate(original_covers) if not ok]
        failed_inset = [indices[k].tolist() for k, ok in enumerate(inset_covers) if not ok]
        audit = {'free_closed_cells_checked': len(indices),
                 'not_covered_by_original_candidate': failed_original,
                 'not_covered_by_numerical_inset': failed_inset,
                 'method': 'GEOSPrepare_r + GEOSPreparedCovers_r on complete closed rectangles',
                 'checks_centers_only': False}
        (output_dir / 'closed_cell_audit.json').write_text(json.dumps(audit, indent=2) + '\n')
        if failed_original or failed_inset:
            raise ValueError('A GDAL free cell failed complete-cell inclusion; no cells were cleared or deleted')

    labels, components = label(bottom_up == 0, structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])
    counts = np.bincount(labels.ravel(), minlength=components+1)[1:]
    component_records = []
    for component_id, (slices, count) in enumerate(zip(find_objects(labels), counts), 1):
        sy, sx = slices
        component_records.append({'id': component_id, 'cells': int(count),
                                  'area_m2': int(count)*dx*dy,
                                  'bounds_m': [x0+sx.start*dx, y0+sy.start*dy,
                                               x0+sx.stop*dx, y0+sy.stop*dy]})
    area = len(indices)*dx*dy
    grid = {'schema_version': 1, 'status': 'closed_cells_covered_by_fixed_candidate',
            'bounds': list(bounds), 'n': n, 'dx_m': dx, 'dy_m': dy,
            'coordinate_convention': 'local projected metres, x east/right, y north/up',
            'rows_bottom_to_top': bottom_up.tolist()}
    (output_dir / 'grid.json').write_text(json.dumps(grid, separators=(',', ':')) + '\n')
    summary = {
        'gdal_version': version, 'GEOS_version': geos.version,
        'source_candidate_area_m2': source_measure['area_m2'],
        'source_candidate_components': source_measure['components'],
        'source_candidate_length_m': source_measure['length_m'],
        'numerical_inset_m': numerical_inset_m,
        'numerical_inset_area_m2': inset_measure['area_m2'],
        'numerical_inset_area_loss_m2': source_measure['area_m2']-inset_measure['area_m2'],
        'free_cells': len(indices), 'blocked_cells': n*n-len(indices),
        'retained_closed_cell_area_m2': area,
        'candidate_area_not_retained_m2': source_measure['area_m2']-area,
        'retained_area_fraction': area/source_measure['area_m2'] if source_measure['area_m2'] else 0.0,
        'connected_components_4': int(components), 'components_4': component_records,
        'all_components_retained': True, 'island_or_hole_filling_applied': False,
        'complete_free_cell_original_covers_passed': not failed_original,
        'complete_free_cell_inset_covers_passed': not failed_inset,
        'north_down_GDAL_transform': info['geoTransform'],
        'rows_bottom_to_top_conversion_checked': True,
        'rasterization': 'ALL_TOUCHED blocking complement of candidate eroded by numerical_inset_m',
        'sources': SOURCES}
    (output_dir / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return grid, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--n', type=int, default=GRID_N,
                        help='Power-of-two grid size; default 1024 gives 5m cells')
    parser.add_argument('--gdal-bin', type=Path)
    parser.add_argument('--geos-lib', type=Path)
    args = parser.parse_args()
    candidate = args.prepared_dir / 'searchable.geojson'
    manifest_file = args.prepared_dir / 'manifest.json'
    candidate_sha = hashlib.sha256(candidate.read_bytes()).hexdigest()
    source_manifest = json.loads(manifest_file.read_text())
    if candidate_sha != source_manifest['derived_files_sha256']['searchable.geojson']:
        raise ValueError('A1 candidate differs from its pinned source manifest')
    document = json.loads(candidate.read_text())
    if len(document['features']) != 1:
        raise ValueError('Expected the fixed single A1 candidate geometry')
    grid, summary = rasterize_closed_cell_grid(
        document['features'][0]['geometry'], document.get('crs'), BOUNDS, args.n,
        args.output_dir, gdal_bin=args.gdal_bin, geos_library=args.geos_lib)
    source = args.output_dir / 'source'; source.mkdir()
    for name in ('searchable.geojson', 'manifest.json', 'config.json', 'summary.json'):
        shutil.copy2(args.prepared_dir / name, source / name)
    snapshot = args.output_dir / 'code_snapshot'; snapshot.mkdir()
    for script in (Path(__file__), Path(__file__).with_name('chart_geometry.py')):
        shutil.copy2(script, snapshot / script.name)
    result = {'generated_utc': datetime.now(timezone.utc).isoformat(),
              'candidate_sha256': candidate_sha,
              'grid_sha256': hashlib.sha256((args.output_dir/'grid.json').read_bytes()).hexdigest(),
              'producer_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'geometry_script_sha256': hashlib.sha256(Path(__file__).with_name('chart_geometry.py').read_bytes()).hexdigest(),
              'A1_manifest_sha256': hashlib.sha256(manifest_file.read_bytes()).hexdigest(),
              'bounds': list(BOUNDS), 'n': grid['n'],
              'dx_m': grid['dx_m'], 'dy_m': grid['dy_m'],
              'summary': summary,
              'output_files_sha256': {str(p.relative_to(args.output_dir)):
                                      hashlib.sha256(p.read_bytes()).hexdigest()
                                      for p in sorted(args.output_dir.rglob('*')) if p.is_file()}}
    (args.output_dir / 'manifest.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('candidate_sha256', 'grid_sha256', 'n', 'dx_m', 'dy_m')}))
    print(json.dumps({k: summary[k] for k in ('free_cells', 'retained_closed_cell_area_m2',
                                           'candidate_area_not_retained_m2', 'connected_components_4')}))
    print(args.output_dir)


if __name__ == '__main__':
    main()
