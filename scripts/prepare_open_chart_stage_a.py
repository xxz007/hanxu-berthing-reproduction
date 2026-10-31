"""Prepare a pinned NOAA ENC pilot using an existing GDAL CLI; never run a planner."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from berthing_planner.coordinates import Mercator
from chart_geometry import Geos

LAYERS = ('LNDARE', 'COALNE', 'DEPARE', 'DEPCNT', 'SLCONS', 'OBSTRN',
          'UWTROC', 'WRECKS', 'M_COVR', 'RESARE', 'CBLARE', 'CBLSUB',
          'PIPSOL', 'PIPARE', 'PRDARE', 'FAIRWY')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/open_chart_long_beach_stage_a.json')
    parser.add_argument('--archives-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--gdal-bin', type=Path)
    parser.add_argument('--geos-lib', type=Path)
    args = parser.parse_args()
    config_bytes = args.config.read_bytes()
    cfg = json.loads(config_bytes)
    gdal_bin = args.gdal_bin
    if gdal_bin is None:
        found = shutil.which('ogr2ogr')
        gdal_bin = Path(found).parent if found else Path('/Applications/Postgres.app/Contents/Versions/18/bin')
    ogr2ogr, ogrinfo = gdal_bin/'ogr2ogr', gdal_bin/'ogrinfo'
    if not ogr2ogr.is_file() or not ogrinfo.is_file():
        raise ValueError('Existing GDAL CLI required; pass --gdal-bin. No package is installed automatically.')
    for chart in cfg['charts']:
        archive = args.archives_dir/(chart['cell']+'.zip')
        if archive.stat().st_size != chart['bytes'] or hashlib.sha256(archive.read_bytes()).hexdigest() != chart['sha256']:
            raise ValueError(f"Pinned archive mismatch: {chart['cell']}; original preserved")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    (args.output_dir/'source').mkdir()
    (args.output_dir/'features').mkdir()
    (args.output_dir/'config.json').write_bytes(config_bytes)
    commands = []

    def execute(command):
        result = subprocess.run([str(x) for x in command], capture_output=True, text=True)
        commands.append({'argv': [str(x) for x in command], 'returncode': result.returncode,
                         'stdout': result.stdout, 'stderr': result.stderr})
        (args.output_dir/'commands.json').write_text(json.dumps(commands, ensure_ascii=False, indent=2)+'\n')
        if result.returncode:
            raise RuntimeError(result.stderr[:1500] or result.stdout[:1500])
        return result.stdout

    version = execute([ogrinfo, '--version']).strip()
    p = cfg['projection']
    projection = Mercator(a=p['a_m'], b=p['b_m'], lon0_deg=p['central_longitude_deg'],
                          lat_ts_deg=p['true_scale_latitude_deg'])
    offset_N = projection.forward(p['central_longitude_deg'], p['origin_latitude_deg'])[1]
    srs = (f"+proj=merc +lat_ts={p['true_scale_latitude_deg']} +lon_0={p['central_longitude_deg']} "
           f"+a={p['a_m']} +b={p['b_m']} +x_0=0 +y_0={-offset_N:.12f} +units=m +no_defs")
    gpkg = args.output_dir/'chart_features.gpkg'
    created_layers = set()
    dsids = []
    for chart in cfg['charts']:
        folder = args.output_dir/'source'/chart['cell']
        folder.mkdir()
        archive = args.archives_dir/(chart['cell']+'.zip')
        with zipfile.ZipFile(archive) as z:
            if any(Path(n).is_absolute() or '..' in Path(n).parts for n in z.namelist()):
                raise ValueError('Invalid archive member path')
            z.extractall(folder)
        base = folder/'ENC_ROOT'/chart['cell']/(chart['cell']+'.000')
        dsid_file = args.output_dir/'source'/(chart['cell']+'_DSID.json')
        execute([ogr2ogr, '-f', 'GeoJSON', dsid_file, base, 'DSID', '-oo', 'UPDATES=APPLY'])
        dsid = json.loads(dsid_file.read_text())['features'][0]['properties']
        if (int(dsid['DSID_EDTN']), int(dsid['DSID_UPDN']), int(dsid['DSPM_CSCL'])) != (
                chart['edition'], chart['update'], chart['scale_denominator']):
            raise ValueError('ENC identification differs from pinned catalog entry')
        if dsid['DSPM_HDAT'] != 2 or dsid['DSPM_DUNI'] != 1:
            raise ValueError('Expected WGS84 geographic coordinates and depth units in metres')
        dsids.append({'cell': chart['cell'], 'properties': dsid})
        available = execute([ogrinfo, '-ro', base, '-oo', 'UPDATES=APPLY'])
        for layer in LAYERS:
            if not any(line.split(': ', 1)[-1].split(' ')[0] == layer for line in available.splitlines()):
                continue
            command = [ogr2ogr, '-f', 'GPKG', gpkg, base, '-oo', 'UPDATES=APPLY',
                       '-sql', f"SELECT *, '{chart['cell']}' AS enc_cell FROM {layer}",
                       '-nln', layer, '-nlt', 'GEOMETRY', '-dim', 'XY', '-unsetFid',
                       '-clipsrc', *map(str, cfg['bbox_lon_lat']), '-t_srs', srs]
            if gpkg.exists():
                command += ['-update']
            if layer in created_layers:
                command += ['-append']
            else:
                command += ['-lco', 'GEOMETRY_NAME=geom']
            execute(command)
            created_layers.add(layer)
    required = set(LAYERS[:9])
    if not required.issubset(created_layers):
        raise ValueError(f'Missing required chart layers: {required-created_layers}')
    for layer in sorted(created_layers):
        execute([ogr2ogr, '-f', 'GeoJSON', args.output_dir/'features'/(layer+'.geojson'),
                 gpkg, layer, '-lco', 'RFC7946=NO'])

    features = {name:json.loads((args.output_dir/'features'/(name+'.geojson')).read_text())['features']
                for name in sorted(created_layers)}
    task = {'type':'Polygon', 'coordinates':[cfg['task_polygon_local_m']]}
    margin = float(cfg['engineering_choices']['obstacle_margin_m'])
    depth = float(cfg['engineering_choices']['minimum_depth_lower_bound_m'])
    with Geos(args.geos_lib) as geos:
        def union(geometries):
            return geos.operation('union', {'type':'GeometryCollection', 'geometries':list(geometries)})

        input_geometry_checks = {}
        for name, fs in features.items():
            checks = [geos.measure(f['geometry']) for f in fs]
            invalid = sum(not c['valid'] for c in checks)
            input_geometry_checks[name] = {'count':len(fs), 'invalid':invalid}
            if invalid:
                raise ValueError(f'Invalid source feature geometry in {name}; no silent repair')
        land = union(f['geometry'] for f in features['LNDARE'] if f['geometry']['type'] in ('Polygon','MultiPolygon'))
        hazards = union(f['geometry'] for name in ('LNDARE','COALNE','SLCONS','OBSTRN','UWTROC','WRECKS') for f in features[name])
        # GEOS also simplifies buffer inputs. Use a declared construction pad
        # and verify the final task-wide distance against the original hazards.
        pad = float(cfg['engineering_choices']['buffer_approximation_pad_m'])
        buffer_distance = (margin+pad)/math.cos(math.pi/(4*30))
        buffered = geos.operation('buffer', hazards, distance=buffer_distance, quadrant_segments=30)
        excluded_codes = set(cfg['engineering_choices']['excluded_restriction_codes'])
        restriction_context = union(f['geometry'] for f in features.get('RESARE',[])
                                    if f['geometry']['type'] in ('Polygon','MultiPolygon'))
        restricted = union(f['geometry'] for f in features.get('RESARE',[])
                           if f['geometry']['type'] in ('Polygon','MultiPolygon')
                           and excluded_codes.intersection(int(c) for c in (f['properties'].get('RESTRN') or [])))
        coverage = union(f['geometry'] for f in features['M_COVR'] if f['properties']['CATCOV']==1)
        no_coverage = union(f['geometry'] for f in features['M_COVR'] if f['properties']['CATCOV']==2)
        depth_qualified = union(f['geometry'] for f in features['DEPARE']
                                if f['properties'].get('DRVAL1') is not None and f['properties']['DRVAL1']>=depth)
        allowed = geos.operation('intersection', coverage, depth_qualified)
        for exclusion in (buffered, restricted, no_coverage):
            allowed = geos.operation('difference', allowed, exclusion)
        search = geos.operation('intersection', task, allowed)
        shapes = {'land':land,'hazards':hazards,'hazard_buffer':buffered,'restricted':restricted,
                  'restriction_context':restriction_context,
                  'coverage':coverage,'no_coverage':no_coverage,'depth_qualified':depth_qualified,
                  'task_boundary':task,'searchable':search,
                  'excluded_task':geos.operation('difference',task,search)}
        summary = {'task_area_m2':geos.measure(task)['area_m2'],
                   'searchable_area_m2':geos.measure(search)['area_m2'],
                   'task_outside_chart_m2':geos.measure(geos.operation('difference',task,coverage))['area_m2'],
                   'forbidden_overlap_m2':geos.measure(geos.operation('intersection',search,buffered))['area_m2'],
                   'task_valid':geos.measure(task)['valid'], 'searchable_valid':geos.measure(search)['valid'],
                   'searchable_components':geos.measure(search)['components'],
                   'minimum_hazard_clearance_m':geos.distance(search,hazards),
                   'task_restriction_context_overlap_m2':geos.measure(geos.operation('intersection',task,restriction_context))['area_m2'],
                   'searchable_restriction_context_overlap_m2':geos.measure(geos.operation('intersection',search,restriction_context))['area_m2'],
                   'buffer_distance_m':buffer_distance,'GEOS_version':geos.version,
                   'source_geometry_checks':input_geometry_checks}
        for name, geometry in shapes.items():
            feature = {'type':'FeatureCollection', 'name':name,
                       'crs':{'type':'name','properties':{'name':srs}},
                       'features':[{'type':'Feature','properties':{},'geometry':geometry}]}
            (args.output_dir/(name+'.geojson')).write_text(json.dumps(feature,ensure_ascii=False,indent=2)+'\n')
    (args.output_dir/'geometry_diagnostics.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    if not summary['task_valid'] or not summary['searchable_valid']:
        raise ValueError('Invalid task or searchable geometry; no silent repair')
    if (summary['searchable_area_m2'] <= 0 or summary['task_outside_chart_m2'] > 0.01
            or summary['forbidden_overlap_m2'] > 0.01 or summary['minimum_hazard_clearance_m'] < margin-1e-6):
        raise ValueError('Task coverage or obstacle exclusion check failed')
    bounds = cfg['bbox_lon_lat']
    x1, n1 = projection.forward(bounds[2], bounds[3])
    summary.update({'bbox_local_m':[0.0,0.0,x1,n1-offset_N], 'projection_proj_string':srs,
                    'feature_counts': {name:len(json.loads((args.output_dir/'features'/(name+'.geojson')).read_text())['features'])
                                       for name in sorted(created_layers)},
                    'task_is_engineering_choice':True, 'coverage_planner_run':False,
                    'depth_is_chart_band_lower_bound':True, 'margin_is_vector_buffer':True,
                    'buffer_segments_per_quadrant':30})
    (args.output_dir/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    (args.output_dir/'source/DSIDs.json').write_text(json.dumps(dsids,ensure_ascii=False,indent=2)+'\n')
    manifest = {'prepared_at_utc':datetime.now(timezone.utc).isoformat(),'gdal_version':version,
                'config_sha256':hashlib.sha256(config_bytes).hexdigest(),
                'preparation_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'geometry_script_sha256':hashlib.sha256((Path(__file__).parent/'chart_geometry.py').read_bytes()).hexdigest(),
                'summary_sha256':hashlib.sha256((args.output_dir/'summary.json').read_bytes()).hexdigest(),
                'charts':cfg['charts'], 'source_updates':'GDAL S57 UPDATES=APPLY; DSID edition/update checked',
                'derived_files_sha256':{str(path.relative_to(args.output_dir)):hashlib.sha256(path.read_bytes()).hexdigest()
                                        for path in args.output_dir.rglob('*.geojson')},
                'source_files_sha256':{str(path.relative_to(args.output_dir)):hashlib.sha256(path.read_bytes()).hexdigest()
                                       for path in (args.output_dir/'source').rglob('*') if path.is_file()},
                'scope':'Map and task only; no path, coverage or control result',
                'raw_data_uploaded':False}
    (args.output_dir/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
