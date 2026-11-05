"""A5 independent source-filter, analytic-arc and obstruction-certificate audit.

No production Dubins, reachable-box or curve-evaluation imports. Native GEOS
checks full independently enclosed curves; NumPy separately checks source masks.
This audits the recorded success/failure experiment, not full mission feasibility.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from berthing_planner.validation.curves import arc_enclosure

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




def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('prepared-dir','navigation-dir','trial-dir','adapted-dir'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--obstructions-json',type=Path,required=True)
    parser.add_argument('--json-out',type=Path,required=True)
    args=parser.parse_args();cfg=json.loads((args.trial_dir/'config.json').read_text())
    trial=json.loads((args.trial_dir/'result.json').read_text());adapted=json.loads((args.adapted_dir/'result.json').read_text())
    proofs=json.loads(args.obstructions_json.read_text());navfile=args.navigation_dir/'navigation.geojson'
    nav=json.loads(navfile.read_text())['features'][0]['geometry'];errors=[];shapes={}
    if hashlib.sha256(navfile.read_bytes()).hexdigest()!=cfg['expected_navigation_sha256']:errors.append('navigation hash')
    for name,digest in cfg['A1_geometry_sha256'].items():
        p=args.prepared_dir/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:errors.append('source hash '+name)
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    for directory in (args.trial_dir,args.adapted_dir):
        provenance=json.loads((directory/'provenance.json').read_text())
        if hashlib.sha256((directory/'result.json').read_bytes()).hexdigest()!=provenance['result_sha256']:errors.append('result hash')
        for p,digest in provenance['input_sha256'].items():
            if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=digest:errors.append('input hash')
        for p,digest in provenance['code_sha256'].items():
            if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=digest:errors.append('code hash '+p)
    source_path=next(Path(p)for p in provenance['input_sha256']if Path(p).name=='result.json')
    source=json.loads(source_path.read_text());science={k:v for k,v in source.items()if k!='total_computation_seconds'}
    if hashlib.sha256(json.dumps(science,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()!=cfg['source_a3_scientific_result_sha256']:errors.append('A3 scientific signature')
    expected_records=set()
    for name in ('uuv_1','uuv_2','usv'):
        legs=source['platforms'][name]['legs']
        expected_records|={(name,i)for i,leg in enumerate(legs)if not leg['scanning']}
    if expected_records!={(r['platform'],r['source_leg_index'])for r in trial['connections']}or len(trial['connections'])!=90:errors.append('source connector record set')
    for r in trial['connections']:
        if r['platform']=='usv':continue
        i=r['source_leg_index'];legs=source['platforms'][r['platform']]['legs'];a,b=legs[i-1],legs[i+1]
        expected_poses=[[*a['points'][-1],math.atan2(a['points'][-1][1]-a['points'][0][1],a['points'][-1][0]-a['points'][0][0])],
                        [*b['points'][0],math.atan2(b['points'][-1][1]-b['points'][0][1],b['points'][-1][0]-b['points'][0][0])]]
        if np.max(np.abs(np.asarray(expected_poses)-np.asarray([r['start_pose'],r['goal_pose']])))>1e-8 or a['task_id']!=r['prior_task_id']or b['task_id']!=r['next_task_id']:errors.append('changed source scan poses')
    axis=np.arange(10,5120,20);xx,yy=np.meshgrid(axis,axis);points=np.column_stack((xx.ravel(),yy.ravel()))
    masks={};boundary=np.zeros(len(points),dtype=bool)
    for name,shape in {**shapes,'navigation':nav}.items():
        masks[name],on=inside(shape,points);boundary|=on
    expected=masks['coverage']&masks['depth_qualified']&~(masks['hazard_buffer']|masks['restricted']|masks['no_coverage'])
    mask_errors=int(np.sum((masks['navigation']!=expected)&~boundary))
    if mask_errors:errors.append('source-filter independent mask mismatch')
    counts={};curve_count=0;min_clearance=math.inf;join_error=heading_error=0.0;proof_count=0
    with Geos()as g:
        target_missing=g.measure(g.operation('difference',shapes['searchable'],nav))['area_m2']
        clearance=g.distance(nav,shapes['hazards'])
        forbidden={name:g.measure(g.operation('intersection',nav,shapes[name]))['area_m2']for name in ('hazard_buffer','restricted','no_coverage')}
        outside_coverage=g.measure(g.operation('difference',nav,shapes['coverage']))['area_m2']
        outside_depth=g.measure(g.operation('difference',nav,shapes['depth_qualified']))['area_m2']
        if max(target_missing,outside_coverage,outside_depth,*forbidden.values())>.001 or clearance<50:errors.append('source safety geometry')
        for label,domain in [('original_domain',shapes['searchable']),('separated_domain',nav)]:
            counts[label]={}
            for record in trial['connections']:
                name=record['platform'];count=counts[label].setdefault(name,{'passed':0,'required':0});count['required']+=1
                path=record[label]
                if not path['passed']:continue
                count['passed']+=1;curve_count+=1;pieces=path['pieces'];radius=record['minimum_radius_m']
                for i,p in enumerate(pieces):
                    if p['direction']!=1 or abs(p['curvature'])>1/radius+1e-12:errors.append('forward curvature bound')
                    if i:
                        prior=pieces[i-1];join_error=max(join_error,float(np.linalg.norm(piece_points(prior,[1])[0]-np.asarray(p['pose'][:2]))))
                        heading_error=max(heading_error,abs(math.remainder(prior['pose'][2]+prior['curvature']*prior['length']-p['pose'][2],2*math.pi)))
                if pieces:
                    endpoint=piece_points(pieces[-1],[1])[0]
                    if np.linalg.norm(endpoint-np.asarray(record['goal_pose'][:2]))>1e-5 or math.dist(pieces[0]['pose'][:2],record['start_pose'][:2])>1e-7:errors.append('connector endpoint')
                    if abs(math.remainder(pieces[0]['pose'][2]-record['start_pose'][2],2*math.pi))>1e-7:errors.append('start heading')
                    if abs(math.remainder(pieces[-1]['pose'][2]+pieces[-1]['curvature']*pieces[-1]['length']-record['goal_pose'][2],2*math.pi))>1e-7:errors.append('goal heading')
                    xy,bound=arc_enclosure(pieces)
                    chord={'type':'LineString','coordinates':xy}
                    envelope=g.operation('buffer',chord,distance=(bound+1e-6)/math.cos(math.pi/120),quadrant_segments=30)
                    if not g.covers(domain,envelope):errors.append('independent whole-curve envelope')
                    min_clearance=min(min_clearance,g.distance(chord,shapes['hazards'])-bound)
                if abs(sum(p['length']for p in pieces)-path['length_m'])>1e-6:errors.append('connector length')
        if counts!=trial['connector_counts']:errors.append('counts differ')
        failures={(r['platform'],r['source_leg_index']):r for r in trial['connections']if not r['separated_domain']['passed']}
        if set(failures)!={(r['platform'],r['source_leg_index'])for r in proofs['records']}:errors.append('obstruction record set')
        for r in proofs['records']:
            original=failures[(r['platform'],r['source_leg_index'])];start,goal=original['start_pose'],original['goal_pose'];radius=original['minimum_radius_m']
            lower=max(math.dist(start[:2],goal[:2]),radius*abs(math.remainder(goal[2]-start[2],2*math.pi)))
            proof_count+=bool(r['certified_endpoint_obstructions'])
            for cert in r['certified_endpoint_obstructions']:
                s=cert['arc_length_m'];p=start if cert['side']=='exit'else[*goal[:2],goal[2]+math.pi]
                if not 0<s<min(lower,math.pi*radius/2):errors.append('invalid certificate length')
                box=np.asarray(cert['reachable_box']['coordinates'][0]);d=box-np.asarray(p[:2]);u=np.array([math.cos(p[2]),math.sin(p[2])]);v=np.array([-u[1],u[0]])
                expected_local=np.array([[radius*math.sin(s/radius),-radius*(1-math.cos(s/radius))],[s,-radius*(1-math.cos(s/radius))],[s,radius*(1-math.cos(s/radius))],[radius*math.sin(s/radius),radius*(1-math.cos(s/radius))],[radius*math.sin(s/radius),-radius*(1-math.cos(s/radius))]])
                if np.max(np.abs(np.column_stack((d@u,d@v))-expected_local))>1e-7:errors.append('reachable outer box')
                distance=g.distance(cert['reachable_box'],nav)
                if distance<=1e-5 or abs(distance-cert['strict_distance_from_navigation_m'])>1e-7:errors.append('certificate separation')
        if proof_count!=proofs['connectors_with_proven_endpoint_obstruction']:errors.append('certificate count')
        # A failed full route must not expose a fabricated mission clock or coverage.
        if adapted['all_platform_motion_geometry_complete']or adapted['full_mission_retiming_released']or adapted['coverage']:errors.append('unexpected full mission claim')
        for name in ('uuv_1','uuv_2'):
            source_ids=[leg['task_id']for leg in source['platforms'][name]['legs']if leg['scanning']]
            if adapted['diagnostics'][name]['task_ids']!=source_ids or len(set(source_ids))!=adapted['platforms'][name]['task_count']:errors.append('lost task identifiers/order')
        usv=adapted['platforms']['usv']
        if usv['motion_geometry_complete']:
            fixed_pieces=[p for r in trial['connections']if r['platform']=='usv'for p in r['separated_domain']['pieces']]
            if usv['pieces']!=fixed_pieces:errors.append('support curve differs from independently checked fixed connections')
            for prior,p in zip(usv['pieces'],usv['pieces'][1:]):
                if np.linalg.norm(piece_points(prior,[1])[0]-p['pose'][:2])>1e-6 or abs(math.remainder(prior['pose'][2]+prior['curvature']*prior['length']-p['pose'][2],2*math.pi))>1e-7:errors.append('support continuity')
            speed_errors=speed_checks(usv['profile'],usv['pieces'],{**cfg,'cruise_mps':cfg['cruise_mps']['usv']})
            errors.extend(speed_errors)
    if join_error>1e-6 or heading_error>1e-7 or min_clearance<50:errors.append('curve continuity/clearance')
    report={'passed':not errors,'errors':errors,'source_result_sha256':hashlib.sha256((args.trial_dir/'result.json').read_bytes()).hexdigest(),
            'adapted_result_sha256':hashlib.sha256((args.adapted_dir/'result.json').read_bytes()).hexdigest(),'navigation_sha256':hashlib.sha256(navfile.read_bytes()).hexdigest(),
            'independent_source_mask_points':len(points),'source_mask_boundary_points_excluded':int(np.sum(boundary)),
            'source_mask_mismatch_count':mask_errors,'target_outside_navigation_m2':target_missing,'navigation_outside_coverage_m2':outside_coverage,
            'navigation_outside_depth_qualified_m2':outside_depth,'navigation_minimum_hazard_clearance_m':clearance,'forbidden_overlap_m2':forbidden,
            'connector_counts':counts,'independent_enclosed_curve_count':curve_count,'minimum_curve_hazard_clearance_lower_bound_m':min_clearance,
            'maximum_join_position_error_m':join_error,'maximum_join_heading_error_rad':heading_error,'independently_verified_obstruction_count':proof_count,
            'full_mission_verified':False,'scope':'Audited fixed-pose experiment and retained failed adaptation; no full-route timing/separation or executed coverage acceptance.'}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
