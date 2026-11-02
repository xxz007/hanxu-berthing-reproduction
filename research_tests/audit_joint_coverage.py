"""Independent A6 full-route arcs, task accounting, speed and footprint audit.

No production curve evaluation, Dubins formulas, variant generation or speed
evaluation imported. Exact point-to-arc distances independently probe coverage.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from chart_geometry import Geos
from audit_open_chart_stage_a import inside
from audit_motion_feasibility import piece_points,speed_checks
from audit_headland_navigation import arc_enclosure


def distance_to_piece(points,piece):
    start=np.asarray(piece['pose'][:2]);end=piece_points(piece,[1])[0];k=piece['curvature']
    if k==0:
        d=end-start;t=np.clip((points-start)@d/(d@d),0,1)
        return np.linalg.norm(points-start-t[:,None]*d,axis=1)
    yaw=piece['pose'][2];centre=start+np.array([-math.sin(yaw)/k,math.cos(yaw)/k]);radius=1/abs(k)
    radial=points-centre;angles=np.arctan2(radial[:,1],radial[:,0]);a0=math.atan2(start[1]-centre[1],start[0]-centre[0])
    progression=(math.copysign(1,k)*(angles-a0))%(2*math.pi)
    within=progression<=abs(k)*piece['length']+1e-12
    endpoints=np.minimum(np.linalg.norm(points-start,axis=1),np.linalg.norm(points-end,axis=1))
    return np.where(within,np.abs(np.linalg.norm(radial,axis=1)-radius),endpoints)


def straight_capsule_upper(geos,points,radius,q=30):
    # Independent explicit circumscribed polygons, not production Point buffers.
    a,b=np.asarray(points);u=(b-a)/np.linalg.norm(b-a);outer=radius+1e-6;normal=np.array([-u[1],u[0]])*outer
    strip={'type':'Polygon','coordinates':[[p.tolist()for p in (a+normal,b+normal,b-normal,a-normal,a+normal)]]}
    angle=np.linspace(0,2*math.pi,4*q+1);unit=np.column_stack((np.cos(angle),np.sin(angle)))*outer/math.cos(math.pi/(4*q))
    unit[-1]=unit[0]
    disks=[{'type':'Polygon','coordinates':[(p+unit).tolist()]}for p in (a,b)]
    return geos.operation('union',{'type':'GeometryCollection','geometries':[strip]+disks})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','a3-run-dir','a5-run-dir','prepared-dir','navigation-dir','json-out'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();x=json.loads((args.run_dir/'result.json').read_text());cfg=json.loads((args.run_dir/'config.json').read_text())
    source=json.loads((args.a3_run_dir/'result.json').read_text());provenance=json.loads((args.run_dir/'provenance.json').read_text());errors=[]
    if hashlib.sha256((args.run_dir/'config.json').read_bytes()).hexdigest()not in provenance['input_sha256'].values():errors.append('copied config hash')
    for p,digest in provenance['input_sha256'].items():
        if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=digest:errors.append('input hash')
    for p,digest in provenance['code_sha256'].items():
        if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=digest:errors.append('code hash '+p)
    for name,digest in provenance['derived_files_sha256'].items():
        if hashlib.sha256((args.run_dir/name).read_bytes()).hexdigest()!=digest:errors.append('derived hash '+name)
    result_hash=hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()
    if result_hash!=provenance['result_sha256']:errors.append('result hash')
    science={k:v for k,v in source.items()if k!='total_computation_seconds'}
    if hashlib.sha256(json.dumps(science,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()!=cfg['source_a3_scientific_result_sha256']:errors.append('source scientific signature')
    shapes={}
    for name in ('searchable','hazards'):
        p=args.prepared_dir/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg['A1_geometry_sha256'][name]:errors.append('source '+name)
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    navpath=args.navigation_dir/'navigation.geojson'
    if hashlib.sha256(navpath.read_bytes()).hexdigest()!=cfg['expected_navigation_sha256']:errors.append('navigation hash')
    nav=json.loads(navpath.read_text())['features'][0]['geometry'];target=shapes['searchable'];hazards=shapes['hazards'];counts={}
    for name in ('uuv_1','uuv_2'):
        tasks=[t for t in source['platforms'][name]['legs']if t['scanning']];variants=json.loads((args.run_dir/(name+'_variants.json')).read_text())
        if set(variants)!={t['task_id']for t in tasks}:errors.append('variant task set')
        oldfile=args.a5_run_dir/(name+'_variants.json')
        if hashlib.sha256(oldfile.read_bytes()).hexdigest()!=cfg['a5_variant_files_sha256'][name]:errors.append('A5 variant hash')
        for oldlayer in json.loads(oldfile.read_text()):
            for v in oldlayer:
                signatures={tuple(round(z,8)for p in w['points']for z in p)for w in variants[v['task_id']]}
                for points in (v['points'],v['points'][::-1]):
                    if tuple(round(z,8)for p in points for z in p)not in signatures:errors.append('discarded A5 candidate')
        counts[name]={'source_tasks':len(tasks),'variant_tasks':len(variants)}
    if not x['complete_platform_routes_found']:
        report={'passed':False,'errors':errors+['No complete routes; full motion/coverage audit withheld'],'task_counts':counts,'source_result_sha256':result_hash}
        args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(report);raise SystemExit(1)
    all_scans=[];all_scans_upper=[];transit_lower=[];transit_upper=[];min_clearance=math.inf;max_join=max_yaw=0.;platform_checks={}
    with Geos()as g:
        for name,plan in x['platforms'].items():
            pieces=plan['pieces'];radius=cfg['usv_minimum_radius_m']if name=='usv'else cfg['uuv_minimum_radius_m'];p_errors=[]
            if not pieces:p_errors.append('empty route')
            for i,p in enumerate(pieces):
                if p['kind']not in ('line','arc')or p['direction']!=1 or p['length']<=0 or abs(p['curvature'])>1/radius+1e-12:p_errors.append('invalid forward primitive')
                if i:
                    prior=pieces[i-1];max_join=max(max_join,float(np.linalg.norm(piece_points(prior,[1])[0]-p['pose'][:2])))
                    max_yaw=max(max_yaw,abs(math.remainder(prior['pose'][2]+prior['curvature']*prior['length']-p['pose'][2],2*math.pi)))
            if abs(sum(p['length']for p in pieces)-plan['length_m'])>1e-5:p_errors.append('route length')
            p_errors.extend(speed_checks(plan['profile'],pieces,{**cfg,'cruise_mps':cfg['cruise_mps'][name]}))
            checklegs=plan['legs']if name!='usv'else[{'kind':'support','scanning':False,'pieces':pieces}]
            for leg in checklegs:
                if not leg['pieces']:
                    if leg['scanning']or abs(leg['length_m'])>1e-9:p_errors.append('nonzero empty leg')
                    continue
                xy,bound=arc_enclosure(leg['pieces']);chord={'type':'LineString','coordinates':xy}
                envelope=g.operation('buffer',chord,distance=(bound+1e-6)/math.cos(math.pi/120),quadrant_segments=30)
                if not g.covers(nav,envelope):p_errors.append('independent full curve enclosure')
                min_clearance=min(min_clearance,g.distance(chord,hazards)-bound)
                if name=='usv':continue
                if leg['scanning']:
                    if len(leg['pieces'])!=1 or leg['pieces'][0]['curvature']!=0:p_errors.append('scan is not declared straight pass')
                    if np.max(np.abs(np.asarray(leg['points'])-np.asarray([leg['pieces'][0]['pose'][:2],piece_points(leg['pieces'][0],[1])[0]])))>1e-6:p_errors.append('scan endpoints differ from primitive')
                    all_scans.append(g.operation('buffer',chord,distance=cfg['sensor_radius_m'],quadrant_segments=30))
                    all_scans_upper.append(straight_capsule_upper(g,leg['points'],cfg['sensor_radius_m']))
                else:
                    transit_lower.append(g.operation('buffer',chord,distance=cfg['sensor_radius_m']-bound,quadrant_segments=30))
                    transit_upper.append(g.operation('buffer',chord,distance=(cfg['sensor_radius_m']+bound)/math.cos(math.pi/120),quadrant_segments=30))
            if name!='usv':
                expected=[t['task_id']for t in source['platforms'][name]['legs']if t['scanning']];actual=[t['task_id']for t in plan['legs']if t['scanning']]
                if sorted(expected)!=sorted(actual)or len(set(actual))!=len(expected):p_errors.append('missing/duplicate task')
                flat=[p for leg in plan['legs']for p in leg['pieces']]
                if flat!=pieces:p_errors.append('legs differ from continuous route')
                counts[name]['executed_scan_passes']=len(actual)
            if p_errors:errors.extend(name+': '+e for e in p_errors)
            platform_checks[name]={'passed':not p_errors,'length_m':plan['length_m'],'independent_motion_duration_s':plan['profile']['duration_s'],'primitive_count':len(pieces)}
        if max_join>1e-6 or max_yaw>1e-7 or min_clearance<50:errors.append('continuity or clearance')
        area=g.measure(target)['area_m2'];covered=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':all_scans}),target)
        lo=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':[covered]+transit_lower}),target)
        scan_hi=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':all_scans_upper}),target)
        hi=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':[scan_hi]+transit_upper}),target)
        ratios={'scan_only_coverage_ratio':g.measure(covered)['area_m2']/area,'scan_only_coverage_upper_ratio':g.measure(scan_hi)['area_m2']/area,'independent_transit_scan_ON_lower_ratio':g.measure(lo)['area_m2']/area,'independent_transit_scan_ON_upper_ratio':g.measure(hi)['area_m2']/area}
        if abs(ratios['scan_only_coverage_ratio']-x['coverage']['scan_only_coverage_ratio'])>1e-10:errors.append('primary footprint mismatch')
        # Independent finer enclosure bounds must overlap the production range.
        if ratios['independent_transit_scan_ON_lower_ratio']>x['coverage']['transit_scan_ON_upper_ratio']+1e-10 or ratios['independent_transit_scan_ON_upper_ratio']<x['coverage']['transit_scan_ON_lower_ratio']-1e-10:errors.append('turn footprint intervals disjoint')
    axis=np.arange(2.5,5120,5);xx,yy=np.meshgrid(axis,axis);sample=np.column_stack((xx.ravel(),yy.ravel()));target_mask,boundary=inside(target,sample);sample=sample[target_mask&~boundary]
    scan_distance=np.full(len(sample),np.inf);all_distance=np.full(len(sample),np.inf)
    for name in ('uuv_1','uuv_2'):
        for leg in x['platforms'][name]['legs']:
            for p in leg['pieces']:
                if p['curvature']:
                    k=p['curvature'];yaw=p['pose'][2];centre=np.asarray(p['pose'][:2])+[-math.sin(yaw)/k,math.cos(yaw)/k];lo=centre-1/abs(k);hi=centre+1/abs(k)
                else:
                    endpoints=np.array([p['pose'][:2],piece_points(p,[1])[0]]);lo=endpoints.min(axis=0);hi=endpoints.max(axis=0)
                subset=np.flatnonzero(np.all((sample>=lo-50.1)&(sample<=hi+50.1),axis=1))
                distance=distance_to_piece(sample[subset],p);all_distance[subset]=np.minimum(all_distance[subset],distance)
                if leg['scanning']:scan_distance[subset]=np.minimum(scan_distance[subset],distance)
    offmask,_=inside(json.loads((args.run_dir/'scan_only_covered.geojson').read_text()),sample)
    onmask,_=inside(json.loads((args.run_dir/'transit_scan_on_covered_lower.geojson').read_text()),sample)
    onupper,_=inside(json.loads((args.run_dir/'transit_scan_on_covered_upper.geojson').read_text()),sample)
    if np.any(offmask&(scan_distance>50+1e-6))or np.any(onmask&(all_distance>50+1e-6)):errors.append('saved covered point beyond exact sensor radius')
    if np.any((scan_distance<50*math.cos(math.pi/120)-1e-6)&~offmask):errors.append('scan footprint unexplained missing sample')
    if np.any((all_distance<50*math.cos(math.pi/120)-2*cfg['maximum_arc_sagitta_m']-1e-6)&~onmask):errors.append('turn footprint unexplained missing sample')
    if np.any((all_distance<50-1e-6)&~onupper):errors.append('exact circular footprint outside saved upper bound')
    if x['full_mission_retiming_released']or x['cooperative_separation_verified']:errors.append('unexpected cooperative clock claim')
    report={'passed':not errors,'errors':errors,'source_result_sha256':result_hash,'task_counts':counts,'platforms':platform_checks,
            'minimum_hazard_clearance_lower_bound_m':min_clearance,'maximum_join_position_error_m':max_join,'maximum_join_heading_error_rad':max_yaw,
            'target_area_m2':area,'coverage':ratios,'independent_sample_spacing_m':5,'independent_target_sample_count':len(sample),'exact_sensor_scan_only_sample_ratio':float(np.mean(scan_distance<=50)),
            'exact_sensor_transit_ON_sample_ratio':float(np.mean(all_distance<=50)),'primary_scan_only_coverage_gate_passed':ratios['scan_only_coverage_ratio']>=cfg['required_coverage_ratio'],
            'transit_scan_ON_sensitivity_gate_passed':min(ratios['independent_transit_scan_ON_lower_ratio'],x['coverage']['transit_scan_ON_lower_ratio'])>=cfg['required_coverage_ratio'],
            'cooperative_timing_and_separation_verified':False,'scope':'Continuous individual2D forward curvature-bounded point routes, declared rest-to-rest speed model and ideal horizontal50m footprint. No cooperative mission clock, hull/dynamics/depth/acoustic verification.'}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
