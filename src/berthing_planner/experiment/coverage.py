"""Run fixed A1 coverage, add explicit missed-area scans, and check vector geometry."""
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.ndimage import label

ROOT = Path(__file__).resolve().parents[3]
from berthing_planner.coverage import plan_coverage, append_sweep
from berthing_planner.occupancy import OccupancyQuadTree
from berthing_planner.experiment.chart_geometry import Geos


def polygons(geometry):
    if geometry['type']=='Polygon':
        return [geometry] if geometry['coordinates'] else []
    if geometry['type']=='MultiPolygon':
        return [{'type':'Polygon','coordinates':p} for p in geometry['coordinates']]
    if geometry['type']=='GeometryCollection':
        return [p for member in geometry['geometries'] for p in polygons(member)]
    return []


def run(*, prepared_dir, grid_dir, config, output_dir):
    cfg=json.loads(config.read_text())
    if not (0<cfg['lane_spacing_m']<=cfg['swath_width_m'] and 0<cfg['target_coverage_ratio']<=1):
        raise ValueError('Require positive spacing <= swath and target coverage in (0,1]')
    target_file=prepared_dir/'searchable.geojson'
    target_hash=hashlib.sha256(target_file.read_bytes()).hexdigest()
    a1_manifest=json.loads((prepared_dir/'manifest.json').read_text())
    grid_manifest=json.loads((grid_dir/'manifest.json').read_text())
    if target_hash != cfg['target_candidate_sha256'] or target_hash != a1_manifest['derived_files_sha256']['searchable.geojson']:
        raise ValueError('A1 candidate geometry changed; do not alter the target to improve coverage')
    observed_a1_manifest_hash=hashlib.sha256((prepared_dir/'manifest.json').read_bytes()).hexdigest()
    config_hash=hashlib.sha256((prepared_dir/'config.json').read_bytes()).hexdigest()
    hazards_hash=hashlib.sha256((prepared_dir/'hazards.geojson').read_bytes()).hexdigest()
    if config_hash!=cfg['a1_config_sha256'] or config_hash!=a1_manifest['config_sha256']:
        raise ValueError('A1 scientific source configuration changed')
    if hazards_hash!=cfg['a1_hazards_sha256'] or hazards_hash!=a1_manifest['derived_files_sha256']['hazards.geojson']:
        raise ValueError('Original A1 hazard geometry changed')
    grid_file=grid_dir/'grid.json'
    if target_hash!=grid_manifest['candidate_sha256'] or hashlib.sha256(grid_file.read_bytes()).hexdigest()!=grid_manifest['grid_sha256']:
        raise ValueError('Planning grid provenance differs from the fixed A1 input')
    grid=json.loads(grid_file.read_text())
    if grid['n']!=cfg['grid_n'] or grid['bounds']!=cfg['grid_bounds']:
        raise ValueError('Grid differs from frozen coverage configuration')
    output_dir.mkdir(parents=True,exist_ok=False)
    (output_dir/'config.json').write_bytes(config.read_bytes())
    (output_dir/'grid_manifest.json').write_bytes((grid_dir/'manifest.json').read_bytes())
    target=json.loads(target_file.read_text())['features'][0]['geometry']
    hazards=json.loads((prepared_dir/'hazards.geojson').read_text())['features'][0]['geometry']
    a1_summary=json.loads((prepared_dir/'summary.json').read_text())
    original_rows=grid['rows_bottom_to_top']
    labels,component_count=label(np.asarray(original_rows)==0,structure=[[0,1,0],[1,1,1],[0,1,0]])
    component_counts=np.bincount(labels.ravel())[1:]
    navigation_component=int(np.argmax(component_counts))+1
    rows=(labels!=navigation_component).astype(np.uint8).tolist()
    navigation_domain={'policy':cfg['scan_center_policy'],'source_grid_components':component_count,
                       'selected_component_id':navigation_component,
                       'selected_free_cells':int(component_counts[navigation_component-1]),
                       'other_free_cells_still_in_coverage_target':int(component_counts.sum()-component_counts[navigation_component-1]),
                       'source_grid_unchanged':True,'target_geometry_unchanged':True}
    navigation_file=output_dir/'navigation_grid.json'
    navigation_file.write_text(json.dumps({**grid,'rows_bottom_to_top':rows,'navigation_domain':navigation_domain},separators=(',',':'))+'\n')
    begin=time.perf_counter()
    tree=OccupancyQuadTree(grid['bounds'],rows)
    tree_seconds=time.perf_counter()-begin
    route_begin=time.perf_counter()
    plan=plan_coverage(tree,rows,lane_spacing_m=cfg['lane_spacing_m'],lane_offset_m=cfg['lane_offset_m'])
    primary_seconds=time.perf_counter()-route_begin
    (output_dir/'primary_plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    primary_leg_count=len(plan['legs'])
    primary_scan_count=plan['scan_leg_count']
    print(json.dumps({'phase':'primary_route','scan_legs':primary_scan_count,'length_m':plan['length_m']}),flush=True)
    free_indices=np.argwhere(np.asarray(rows)==0)
    dx,dy=grid['dx_m'],grid['dy_m']
    x0,y0=grid['bounds'][:2]
    free_points=np.column_stack((x0+(free_indices[:,1]+.5)*dx,y0+(free_indices[:,0]+.5)*dy))
    radius=cfg['swath_width_m']/2
    empty={'type':'GeometryCollection','geometries':[]}
    repairs=[]
    with Geos() as geos:
        def union(geometries):
            return geos.operation('union',{'type':'GeometryCollection','geometries':list(geometries)})

        def footprint(leg):
            points=leg['points']
            shape=({'type':'Point','coordinates':points[0]} if points[0]==points[-1]
                   else {'type':'LineString','coordinates':points})
            buffered=geos.operation('buffer',shape,distance=radius,
                                    quadrant_segments=cfg['footprint_quadrant_segments'])
            return geos.operation('intersection',buffered,target)

        target_area=geos.measure(target)['area_m2']
        if abs(target_area-a1_summary['searchable_area_m2'])>1e-6:
            raise ValueError('Target area differs from frozen A1 summary')
        footprints=[footprint(leg) for leg in plan['legs'] if leg['scanning']]
        covered=union(footprints)
        missed=geos.operation('difference',target,covered)
        initial_coverage=geos.measure(covered)['area_m2']/target_area
        initial_missed_area=geos.measure(missed)['area_m2']
        initial_missed=missed
        print(json.dumps({'phase':'initial_coverage','coverage_ratio':initial_coverage,'missed_area_m2':initial_missed_area}),flush=True)
        for iteration in range(cfg['max_supplemental_scans']):
            if geos.measure(covered)['area_m2']/target_area>=cfg['target_coverage_ratio']:
                break
            parts=polygons(missed)
            if not parts:
                break
            selected=None
            for rank,part in enumerate(sorted(parts,key=lambda p:geos.measure(p)['area_m2'],reverse=True),1):
                point=geos.operation('point_on_surface',part)['coordinates']
                nearest=int(np.argmin(np.sum((free_points-np.asarray(point))**2,axis=1)))
                j,i=map(int,free_indices[nearest])
                left=right=i
                reach=max(1,int(math.ceil(cfg['supplemental_half_length_m']/dx)))
                while left>max(0,i-reach) and rows[j][left-1]==0:
                    left-=1
                while right<min(grid['n']-1,i+reach) and rows[j][right+1]==0:
                    right+=1
                start=[x0+(left+.5)*dx,y0+(j+.5)*dy]
                end=[x0+(right+.5)*dx,y0+(j+.5)*dy]
                if start==end:
                    lower=upper=j
                    reach=max(1,int(math.ceil(cfg['supplemental_half_length_m']/dy)))
                    while lower>max(0,j-reach) and rows[lower-1][i]==0:
                        lower-=1
                    while upper<min(grid['n']-1,j+reach) and rows[upper+1][i]==0:
                        upper+=1
                    start=[x0+(i+.5)*dx,y0+(lower+.5)*dy]
                    end=[x0+(i+.5)*dx,y0+(upper+.5)*dy]
                prospective=footprint({'points':[start,end]})
                gain=geos.measure(geos.operation('intersection',prospective,missed))['area_m2']
                if gain>1e-7:
                    selected=(part,point,nearest,start,end,prospective,rank)
                    break
            if selected is None:
                break
            largest,point,nearest,start,end,new_footprint,rank=selected
            if math.dist(plan['path'][-1],end)<math.dist(plan['path'][-1],start):
                start,end=end,start
            old_area=geos.measure(covered)['area_m2']
            append_sweep(tree,plan,start,end,kind='supplement')
            footprints.append(new_footprint)
            covered=union((covered,new_footprint))
            missed=geos.operation('difference',target,covered)
            gained=geos.measure(covered)['area_m2']-old_area
            repairs.append({'iteration':iteration+1,'missed_polygon_area_m2':geos.measure(largest)['area_m2'],
                            'missed_polygon_area_rank':rank,
                            'representative_point':point,'nearest_free_centre':free_points[nearest].tolist(),
                            'scan_start':start,'scan_end':end,'added_covered_area_m2':gained,
                            'coverage_ratio':geos.measure(covered)['area_m2']/target_area})
            (output_dir/'repair_progress.json').write_text(json.dumps(repairs,ensure_ascii=False,indent=2)+'\n')
            print(json.dumps({'phase':'supplement','iteration':iteration+1,'added_area_m2':gained,'coverage_ratio':repairs[-1]['coverage_ratio']}),flush=True)
            if gained<=1e-7:
                break
        repeated,seen=empty,empty
        pass_area=0.0
        for scan in footprints:
            overlap=geos.operation('intersection',seen,scan)
            repeated=union((repeated,overlap))
            seen=union((seen,scan))
            pass_area+=geos.measure(scan)['area_m2']
        area=geos.measure(covered)['area_m2']
        repeated_area=geos.measure(repeated)['area_m2']
        missed_area=geos.measure(missed)['area_m2']
        route_geometry={'type':'LineString','coordinates':plan['path']}
        route_inside=geos.covers(target,route_geometry)
        clearance=geos.distance(route_geometry,hazards)
        vector_coverage={'target_geometry_sha256':target_hash,'target_area_m2':target_area,
                         'covered_area_m2':area,'missed_area_m2':missed_area,'repeated_area_m2':repeated_area,
                         'coverage_ratio':area/target_area,'missed_ratio':missed_area/target_area,
                         'repeated_ratio':repeated_area/target_area,'summed_pass_area_m2':pass_area,
                         'excess_area_m2':pass_area-area,'excess_fraction_of_pass_area':(pass_area-area)/pass_area,
                         'initial_coverage_ratio':initial_coverage,'initial_missed_area_m2':initial_missed_area,
                         'sensor_model':cfg['sensor_model'],'buffer_quadrant_segments':cfg['footprint_quadrant_segments'],
                         'circle_approximation':'Inscribed polygonal caps; vector footprint area is a conservative approximation of ideal radius50m circles'}
        checks={'target_unchanged':True,'full_path_inside_candidate':route_inside,
                'minimum_original_hazard_clearance_m':clearance,
                'coverage_target_passed':area/target_area>=cfg['target_coverage_ratio'],
                'partition_area_residual_m2':abs(area+missed_area-target_area),
                'full_path_length_recomputed_m':geos.measure(route_geometry)['length_m'],
                'quadtree_LOS_all_segments':all(tree.line_of_sight(a,b) for a,b in zip(plan['path'],plan['path'][1:]))}
        passed=(route_inside and clearance>=cfg['acceptance']['minimum_original_hazard_clearance_m']-1e-6
                and checks['coverage_target_passed'] and checks['quadtree_LOS_all_segments']
                and checks['partition_area_residual_m2']<.01
                and abs(checks['full_path_length_recomputed_m']-plan['length_m'])<1e-6)
        component_coverage=[]
        for component_id,count in enumerate(component_counts,1):
            if component_id==navigation_component:
                continue
            cells=np.argwhere(labels==component_id)
            rectangles=[{'type':'Polygon','coordinates':[[[x0+i*dx,y0+j*dy],[x0+(i+1)*dx,y0+j*dy],
                         [x0+(i+1)*dx,y0+(j+1)*dy],[x0+i*dx,y0+(j+1)*dy],[x0+i*dx,y0+j*dy]]]} for j,i in cells]
            region=union(rectangles)
            component_area=geos.measure(region)['area_m2']
            component_covered=geos.measure(geos.operation('intersection',region,covered))['area_m2']
            component_coverage.append({'source_grid_component':component_id,'closed_cells':int(count),
                                       'closed_cell_area_m2':component_area,'covered_area_m2':component_covered,
                                       'missed_area_m2':component_area-component_covered,
                                       'coverage_ratio':component_covered/component_area,
                                       'platform_centres_visited':False,'retained_in_original_coverage_target':True})
        result={**plan,'status':'geometric_coverage_passed' if passed else 'geometric_coverage_failed',
                'target_geometry_sha256':target_hash,'vector_coverage':vector_coverage,'checks':checks,
                'navigation_domain':navigation_domain,'other_navigation_component_coverage':component_coverage,
                'primary_scan_leg_count':primary_scan_count,'primary_leg_count':primary_leg_count,
                'supplemental_scan_count':len(repairs),'repairs':repairs,
                'primary_planning_seconds':primary_seconds,'tree_seconds':tree_seconds,
                'total_experiment_seconds':time.perf_counter()-begin,
                'motion_constraints_verified':False,'multi_platform_run':False}
        (output_dir/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        for name,geometry in (('covered',covered),('missed',missed),('repeated',repeated),('initial_missed',initial_missed)):
            (output_dir/(name+'.geojson')).write_text(json.dumps({'type':'FeatureCollection','features':[
                {'type':'Feature','properties':{},'geometry':geometry}]},ensure_ascii=False,indent=2)+'\n')
        sources=[config,target_file,prepared_dir/'manifest.json',grid_file,grid_dir/'manifest.json',navigation_file,
                 Path(__file__),ROOT/'src/berthing_planner/coverage.py',ROOT/'src/berthing_planner/occupancy.py',
                 ROOT/'src/berthing_planner/search.py',ROOT/'src/berthing_planner/experiment/chart_geometry.py']
        source_dir=output_dir/'code_snapshot';source_dir.mkdir()
        for path in sources[-5:]:
            destination=source_dir/path.relative_to(ROOT)
            destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(path.read_bytes())
        provenance={'recorded_utc':datetime.now(timezone.utc).isoformat(),
                    'A1_reference_manifest_sha256':cfg['a1_manifest_sha256'],
                    'observed_A1_manifest_sha256':observed_a1_manifest_hash,
                    'A1_manifest_matches_reference':observed_a1_manifest_hash==cfg['a1_manifest_sha256'],
                    'scientific_target_hazards_and_config_unchanged':True,
                    'input_and_code_sha256':{str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
                    'result_sha256':hashlib.sha256((output_dir/'result.json').read_bytes()).hexdigest(),
                    'geometry_sha256':{path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in output_dir.glob('*.geojson')},
                    'GEOS_version':geos.version,'raw_map_uploaded':False,'curvature_or_control_verified':False}
        (output_dir/'provenance.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({'status':result['status'],'length_m':plan['length_m'],
                          'primary_scans':primary_scan_count,'supplemental_scans':len(repairs),
                          'coverage':vector_coverage,'checks':checks},ensure_ascii=False,indent=2))
        if not passed:
            raise SystemExit(1)
