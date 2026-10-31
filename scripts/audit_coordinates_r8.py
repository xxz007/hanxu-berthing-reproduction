"""Validate the standard-library coordinate kernel against existing PROJ.

No library installation, network access, or change to original source material.
An available official PROJ executable is required for the independent check.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from berthing_planner.coordinates import (
    GridTransform, Mercator, thesis_grid_literal, thesis_inverse_literal)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proj-executable', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    executable = args.proj_executable or shutil.which('proj')
    if executable is None:
        candidates = sorted(Path('/Applications/Postgres.app/Contents/Versions').glob('*/bin/proj'))
        executable = candidates[-1] if candidates else None
    if executable is None:
        raise FileNotFoundError('Existing PROJ is required; pass --proj-executable without installing packages')
    executable = Path(executable).resolve()
    output = args.output_dir or ROOT/'outputs'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_coordinates_r8')
    output.mkdir(parents=True, exist_ok=False)
    version_probe = subprocess.run([str(executable)], capture_output=True, text=True)
    version = (version_probe.stderr or version_probe.stdout).splitlines()[0]
    rng = random.Random(20261030)
    fixed = [(0,0), (1,0), (0,1), (103.6,1.1), (103.8,1.2), (104.1,1.5),
             (12,45), (-75,-45), (179,85), (-179,-85), (0,89.9), (0,-89.9)]
    singapore = [(rng.uniform(103.4,104.5), rng.uniform(.8,1.7)) for _ in range(200)]
    global_points = [(rng.uniform(-179,179), rng.uniform(-85,85)) for _ in range(200)]
    points = fixed+singapore+global_points
    projection_configs = [
        {'name':'thesis_B0_L0_zero', 'args':{}},
        {'name':'local_meridian_true_scale', 'args':{'lon0_deg':103.8,'lat_ts_deg':1.2}},
        {'name':'nonzero_standard_parallel', 'args':{'lon0_deg':103,'lat_ts_deg':30}},
        {'name':'WGS84_sensitivity_only', 'args':{'b':6356752.314245179}}]
    results = []
    for config in projection_configs:
        projection = Mercator(**config['args'])
        command = [str(executable), '-f', '%.12f', '+proj=merc',
                   f'+a={projection.a:.15g}', f'+b={projection.b:.15g}',
                   f'+lon_0={projection.lon0_deg:.15g}', f'+lat_ts={projection.lat_ts_deg:.15g}']
        inputs = ''.join(f'{lon:.17g} {lat:.17g}\n' for lon,lat in points)
        forward = subprocess.run(command, input=inputs, text=True, capture_output=True, check=True)
        refs = [tuple(map(float, line.split()[:2])) for line in forward.stdout.splitlines()]
        assert len(refs)==len(points)
        inverse_input = ''.join(f'{E:.17g} {N:.17g}\n' for E,N in refs)
        inverse = subprocess.run(command[:1]+['-I']+command[1:], input=inverse_input,
                                 text=True, capture_output=True, check=True)
        inverse_refs = [tuple(map(float, line.split()[:2])) for line in inverse.stdout.splitlines()]
        records = []
        for (lon,lat),(E,N),(ref_lon,ref_lat) in zip(points,refs,inverse_refs):
            actual_E,actual_N = projection.forward(lon,lat)
            actual_lon,actual_lat = projection.inverse(E,N)
            records.append({
                'lon_deg':lon, 'lat_deg':lat, 'proj_E_m':E, 'proj_N_m':N,
                'forward_error_m':math.hypot(actual_E-E,actual_N-N),
                'inverse_error_deg':max(abs(math.remainder(actual_lon-ref_lon,360)),abs(actual_lat-ref_lat)),
                'roundtrip_error_deg':max(abs(math.remainder(actual_lon-lon,360)),abs(actual_lat-lat))})
        maximum_forward = max(r['forward_error_m'] for r in records)
        maximum_inverse = max(r['inverse_error_deg'] for r in records)
        result = {'name':config['name'], 'parameters':{k:v for k,v in asdict(projection).items()
                                                   if not k.startswith('_')},
                  'point_count':len(points), 'maximum_forward_error_m':maximum_forward,
                  'maximum_inverse_error_deg':maximum_inverse,
                  'passed':maximum_forward<1e-6 and maximum_inverse<1e-9,
                  'proj_command':command, 'points':records}
        results.append(result)
        (output/(config['name']+'.json')).write_text(json.dumps(result,indent=2)+'\n')

    projection = Mercator()
    E,N = projection.forward(103.8,1.2)
    literal_lon,literal_lat = thesis_inverse_literal(E,N)
    scale_audit = []
    e2 = math.sqrt(projection.a**2-projection.b**2)/projection.b
    for B0 in (0,1.2,30,60):
        phi = math.radians(B0)
        literal = (projection.a**2/projection.b)*math.cos(phi)/math.sqrt(1+e2**2*math.cos(phi))
        standard = Mercator(lat_ts_deg=B0)._K
        scale_audit.append({'B0_deg':B0,'literal_printed_K_m':literal,'standard_K_m':standard,
                            'difference_m':literal-standard})
    grid = GridTransform(1000,2000,20,50)
    formula_audit = {
        'eq_3_7_forward_xy':'x is E from longitude; y is N from latitude',
        'eq_3_8_literal_axis_swap_example':{
            'original_lon_lat_deg':[103.8,1.2], 'forward_x_y_m':[E,N],
            'literal_inverse_lon_lat_deg':[literal_lon,literal_lat],
            'corrected_inverse_lon_lat_deg':projection.inverse(E,N)},
        'eq_3_7_printed_cos_vs_cos_squared':scale_audit,
        'eq_3_9_units_example':{
            'source_world_EN_m':[1060,2100], 'origin_EN_m':[1000,2000],
            'cell_size_m':[20,50], 'correct_grid_col_row':grid.world_to_grid(1060,2100),
            'literal_using_m_per_cell':thesis_grid_literal(1060,2100,1000,2000,20,50),
            'literal_using_cell_per_m':thesis_grid_literal(1060,2100,1000,2000,1/20,1/50)},
        'actual_implementation':'Standard E/N; explicit m/cell division and row sign; paper literal variants audit-only'}
    (output/'formula_audit.json').write_text(json.dumps(formula_audit,indent=2)+'\n')
    files = [ROOT/'src/berthing_planner/coordinates.py',ROOT/'tests/test_coordinates.py',
             Path(__file__).resolve(),ROOT/'docs/r8_coordinate_sources.json']
    snapshot = output/'code_snapshot'
    hashes = {}
    for path in files:
        relative = path.relative_to(ROOT);target = snapshot/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,target);hashes[str(relative)] = hashlib.sha256(path.read_bytes()).hexdigest()
    summary = {
        'generated_utc':datetime.now(timezone.utc).isoformat(),
        'status':'passed' if all(r['passed'] for r in results) else 'failed',
        'python':platform.python_version(),'proj_version':version,
        'proj_executable_sha256':hashlib.sha256(executable.read_bytes()).hexdigest(),
        'projection_configurations':len(results),'total_reference_comparisons':sum(r['point_count'] for r in results),
        'maximum_forward_error_m':max(r['maximum_forward_error_m'] for r in results),
        'maximum_inverse_error_deg':max(r['maximum_inverse_error_deg'] for r in results),
        'forward_tolerance_m':1e-6,'inverse_tolerance_deg':1e-9,
        'source_sha256':hashes,'thesis_ellipsoid_preserved':True,
        'author_implementation_equivalence_claimed':False,
        'scope':'Coordinate kernel and formula audit only; not GSHHG/replanning or full chapter acceptance'}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ('status','proj_version','total_reference_comparisons',
          'maximum_forward_error_m','maximum_inverse_error_deg')},ensure_ascii=False))
    print(output)
    if summary['status']!='passed':
        raise AssertionError('Coordinate kernel disagrees with independent PROJ')


if __name__=='__main__':
    main()
