"""Reconstruct safe chart water before task clipping; coverage target unchanged."""
import argparse
import hashlib
import json
from pathlib import Path

from chart_geometry import Geos
from prepare_coverage_grid import rasterize_closed_cell_grid

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/headland_navigation_a5.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--with-grid',action='store_true')
    args=parser.parse_args();cfg=json.loads(args.config.read_text())
    manifest=json.loads((args.prepared_dir/'manifest.json').read_text());shapes={};source_hashes={}
    for name,expected in cfg['A1_geometry_sha256'].items():
        path=args.prepared_dir/(name+'.geojson');digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if digest!=expected or manifest['derived_files_sha256'][path.name]!=digest:raise ValueError('A1 source changed: '+name)
        document=json.loads(path.read_text());shapes[name]=document['features'][0]['geometry'];source_hashes[name]=digest
    args.output_dir.mkdir(parents=True,exist_ok=False)
    (args.output_dir/'config.json').write_bytes(args.config.read_bytes())
    bounds=cfg['navigation_bounds_m'];x0,y0,x1,y1=bounds
    box={'type':'Polygon','coordinates':[[[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]]]}
    with Geos()as g:
        allowed=g.operation('intersection',shapes['coverage'],shapes['depth_qualified'])
        for name in ('hazard_buffer','restricted','no_coverage'):allowed=g.operation('difference',allowed,shapes[name])
        allowed=g.operation('intersection',allowed,box)
        # The original task is mathematically a subset of allowed water. Union
        # retains its exact boundary against Boolean-operation round-off.
        nav=g.operation('union',{'type':'GeometryCollection','geometries':[allowed,shapes['searchable']]})
        if not g.covers(box,nav):raise ValueError('Navigation extends outside declared chart/grid bounds')
        clearance=g.distance(nav,shapes['hazards'])
        target_outside=g.measure(g.operation('difference',shapes['searchable'],nav))['area_m2']
        forbidden={name:g.measure(g.operation('intersection',nav,shapes[name]))['area_m2']for name in ('hazard_buffer','restricted','no_coverage')}
        if not g.measure(nav)['valid']or clearance<cfg['minimum_original_hazard_clearance_m']or target_outside>.001 or max(forbidden.values())>.001:
            raise ValueError('Reconstructed navigation violates unchanged source safety filters')
        result={'navigation_area_m2':g.measure(nav)['area_m2'],'coverage_target_area_m2':g.measure(shapes['searchable'])['area_m2'],
                'task_outside_navigation_m2':target_outside,'minimum_original_hazard_clearance_m':clearance,
                'navigation_outside_target_m2':g.measure(g.operation('difference',nav,shapes['searchable']))['area_m2'],
                'forbidden_overlap_m2':forbidden,'original_target_sha256':source_hashes['searchable'],
                'source_geometry_sha256':source_hashes,'navigation_safety_policy':'Original depth-qualified covered chart water, minus original50m buffered physical hazards and explicit restrictions, without task clipping',
                'GEOS_version':g.version}
    doc={'type':'FeatureCollection','crs':document['crs'],'features':[{'type':'Feature','properties':{},'geometry':nav}]}
    (args.output_dir/'navigation.geojson').write_text(json.dumps(doc,indent=2)+'\n')
    if args.with_grid:
        grid,stats=rasterize_closed_cell_grid(nav,document['crs'],bounds,cfg['grid_n'],args.output_dir/'grid')
        result['grid_summary']=stats;result['grid_sha256']=hashlib.sha256((args.output_dir/'grid/grid.json').read_bytes()).hexdigest()
    result['navigation_sha256']=hashlib.sha256((args.output_dir/'navigation.geojson').read_bytes()).hexdigest()
    if result['navigation_sha256']!=cfg['expected_navigation_sha256']:raise ValueError('Navigation reconstruction differs from fixed A5 geometry')
    (args.output_dir/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items()if k not in ('source_geometry_sha256','grid_summary')},indent=2))


if __name__=='__main__':main()
