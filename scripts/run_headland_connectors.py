"""Try all forward Dubins connector families in fixed and separated domains.

Source scan tasks are unchanged. Missing connectors remain explicit failures;
no discontinuous fragments are published as a complete trajectory.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import line
from berthing_planner.dubins import candidates
from berthing_planner.motion_feasibility import stopping_profile
from chart_geometry import Geos
from run_motion_feasibility import sample_enclosure


def certify(geos,domain,hazards,pieces,tolerance):
    if not pieces:return {'passed':True,'points':[],'maximum_chord_error_m':0.0}
    points,error=sample_enclosure(pieces,tolerance)
    chord={'type':'LineString','coordinates':points}
    envelope=geos.operation('buffer',chord,distance=(error+1e-5)/math.cos(math.pi/120),quadrant_segments=30)
    inside=geos.covers(domain,envelope)
    return {'passed':bool(inside),'points':points,'maximum_chord_error_m':error,
            'minimum_hazard_clearance_lower_bound_m':geos.distance(hazards,chord)-error if inside else None}


def trial(geos,domain,hazards,start,goal,radius,tolerance):
    rejected=[]
    for candidate in candidates(start,goal,radius):
        check=certify(geos,domain,hazards,candidate['pieces'],tolerance)
        if check['passed']and(check.get('minimum_hazard_clearance_lower_bound_m')is None or check['minimum_hazard_clearance_lower_bound_m']>=50):
            return {'passed':True,'word':candidate['word'],'length_m':candidate['length_m'],
                    'pieces':[asdict(p)for p in candidate['pieces']],'certificate':check,'rejected_shorter_words':rejected}
        rejected.append({'word':candidate['word'],'length_m':candidate['length_m']})
    return {'passed':False,'rejected_words':rejected,'search_scope':'six direct forward Dubins words; not a global infeasibility certificate'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a3-run-dir',type=Path,required=True)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--navigation-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/headland_navigation_a5.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args();cfg=json.loads(args.config.read_text())
    source=json.loads((args.a3_run_dir/'result.json').read_text());science={k:v for k,v in source.items()if k!='total_computation_seconds'}
    if hashlib.sha256(json.dumps(science,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()!=cfg['source_a3_scientific_result_sha256']:
        raise ValueError('A3 source scientific result changed')
    shapes={}
    for name in ('searchable','hazards'):
        path=args.prepared_dir/(name+'.geojson')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=cfg['A1_geometry_sha256'][name]:raise ValueError('A1 shape changed')
        shapes[name]=json.loads(path.read_text())['features'][0]['geometry']
    navfile=args.navigation_dir/'navigation.geojson';navsummary=json.loads((args.navigation_dir/'summary.json').read_text())
    if hashlib.sha256(navfile.read_bytes()).hexdigest()!=navsummary['navigation_sha256']:raise ValueError('Navigation changed')
    if navsummary['navigation_sha256']!=cfg['expected_navigation_sha256']:raise ValueError('Navigation does not match fixed A5 geometry')
    shapes['navigation']=json.loads(navfile.read_text())['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False);(args.output_dir/'config.json').write_bytes(args.config.read_bytes())
    connections=[]
    with Geos()as g:
        for name in ('uuv_1','uuv_2'):
            legs=source['platforms'][name]['legs']
            for i,leg in enumerate(legs):
                if leg['scanning']:continue
                prior,following=legs[i-1],legs[i+1];a,b=prior['points'],following['points']
                start=(*a[-1],math.atan2(a[-1][1]-a[0][1],a[-1][0]-a[0][0]))
                goal=(*b[0],math.atan2(b[-1][1]-b[0][1],b[-1][0]-b[0][0]))
                record={'platform':name,'source_leg_index':i,'prior_task_id':prior['task_id'],'next_task_id':following['task_id'],
                        'start_pose':start,'goal_pose':goal,'minimum_radius_m':cfg['uuv_minimum_radius_m'],
                        'original_transit_points':leg['points'],'original_transit_length_m':leg['length_m']}
                for domain,label in [(shapes['searchable'],'original_domain'),(shapes['navigation'],'separated_domain')]:
                    record[label]=trial(g,domain,shapes['hazards'],start,goal,cfg['uuv_minimum_radius_m'],cfg['maximum_arc_sagitta_m'])
                connections.append(record)
            print(name,'connectors checked',flush=True)
        knots=[source['platforms']['usv']['legs'][0]['points'][0]]+[leg['points'][-1]for leg in source['platforms']['usv']['legs']]
        headings=[]
        for i,p in enumerate(knots):
            a,b=knots[max(0,i-1)],knots[min(len(knots)-1,i+1)]
            headings.append(math.atan2(b[1]-a[1],b[0]-a[0]))
        for i in range(len(knots)-1):
            start=(*knots[i],headings[i]);goal=(*knots[i+1],headings[i+1]);record={'platform':'usv','source_leg_index':i,
                'start_pose':start,'goal_pose':goal,'minimum_radius_m':cfg['usv_minimum_radius_m']}
            for domain,label in [(shapes['searchable'],'original_domain'),(shapes['navigation'],'separated_domain')]:
                record[label]=trial(g,domain,shapes['hazards'],start,goal,cfg['usv_minimum_radius_m'],cfg['maximum_arc_sagitta_m'])
            connections.append(record)
    counts={label:{name:{'passed':sum(r[label]['passed']for r in connections if r['platform']==name),
                        'required':sum(r['platform']==name for r in connections)}for name in ('uuv_1','uuv_2','usv')}
            for label in ('original_domain','separated_domain')}
    full=all(r['separated_domain']['passed']for r in connections)
    result={'status':'fixed_scan_connector_trial_completed','scan_tasks_unchanged':True,'coverage_target_sha256':cfg['A1_geometry_sha256']['searchable'],
            'planned_scan_footprints_if_executed':source['coverage'],'connector_counts':counts,'connections':connections,
            'all_direct_connectors_found':full,'full_mission_retiming_released':False,
            'interpretation':'Six direct forward Dubins families; failures require further planning or explicit task adaptation. Planned footprints are not a claim of executed coverage.'}
    (args.output_dir/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    files=[args.config,args.a3_run_dir/'result.json',args.prepared_dir/'searchable.geojson',args.prepared_dir/'hazards.geojson',args.navigation_dir/'navigation.geojson']
    code=[Path(__file__),ROOT/'src/berthing_planner/dubins.py',ROOT/'src/berthing_planner/curves.py',ROOT/'src/berthing_planner/headland.py',ROOT/'src/berthing_planner/motion_feasibility.py',ROOT/'scripts/run_headland_connectors.py',ROOT/'scripts/run_motion_feasibility.py',ROOT/'scripts/chart_geometry.py']
    provenance={'input_sha256':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in files},'code_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in code},'result_sha256':hashlib.sha256((args.output_dir/'result.json').read_bytes()).hexdigest()}
    (args.output_dir/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps({'connector_counts':counts,'all_direct_connectors_found':full},indent=2))


if __name__=='__main__':main()
