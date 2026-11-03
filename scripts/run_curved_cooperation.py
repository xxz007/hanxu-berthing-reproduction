"""A7 common clock with USV transfer-to-station support and continuous bounds."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece
from berthing_planner.curved_timing import CurvedClock,continuous_distance_bounds
from berthing_planner.motion_feasibility import convex_hull,stopping_profile
from berthing_planner.dubins import candidates
from navigation_geometry import NavigationGeometry
from run_motion_feasibility import sample_enclosure


def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a6-run-dir','repair-dir','route-audit-json','repair-audit-json','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/curved_cooperation_a7.json')
    args=parser.parse_args();cfg=json.loads(args.config.read_text());started=time.perf_counter()
    sourcefile=args.a6_run_dir/'result.json';repairfile=args.repair_dir/'result.json';x=json.loads(sourcefile.read_text());repair=json.loads(repairfile.read_text());route_audit=json.loads(args.route_audit_json.read_text());repair_audit=json.loads(args.repair_audit_json.read_text())
    source_hash=hashlib.sha256(sourcefile.read_bytes()).hexdigest();repair_hash=hashlib.sha256(repairfile.read_bytes()).hexdigest()
    if canonical({k:v for k,v in x.items()if k!='total_computation_seconds'})!=cfg['source_a6_scientific_sha256']or canonical({k:v for k,v in repair.items()if k!='source_result_sha256'})!=cfg['source_repair_scientific_sha256']:raise ValueError('Frozen A6 science changed')
    if not route_audit['passed']or route_audit['source_result_sha256']!=source_hash or not repair_audit['passed']or repair_audit['route_source_result_sha256']!=source_hash or repair_audit['repair_source_result_sha256']!=repair_hash:raise ValueError('Matching A6 audits required')
    shapes={};files=[args.config,sourcefile,repairfile,args.route_audit_json,args.repair_audit_json]
    for name,key,directory in [('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir),('navigation','navigation_sha256',args.navigation_dir)]:
        path=directory/(name+'.geojson');files.append(path)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Frozen chart geometry changed: '+name)
        shapes[name]=json.loads(path.read_text())['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False);(args.output_dir/'config.json').write_bytes(args.config.read_bytes());(args.output_dir/'certificates').mkdir()
    settings={'tolerance_m':cfg['distance_tolerance_m'],'numerical_margin_m':cfg['distance_numerical_margin_m'],'max_refinements':cfg['max_distance_refinements']}
    def certify(first,second,end,label):
        report=continuous_distance_bounds(first,second,end,**settings);path='certificates/'+label+'.json';(args.output_dir/path).write_text(json.dumps(report,indent=2)+'\n')
        return {k:v for k,v in report.items()if k!='partition'}|{'certificate_file':path}
    naive={name:CurvedClock(p['pieces'],p['profile'])for name,p in x['platforms'].items()};naive_end=max(c.finish_s for c in naive.values());naive_pairs={}
    for a,b in [('uuv_1','uuv_2'),('usv','uuv_1'),('usv','uuv_2')]:naive_pairs[a+'__'+b]=certify(naive[a],naive[b],naive_end,'naive_'+a+'__'+b)
    def passes(pairs):return all(p['converged']and p['minimum_distance_lower_m']>=cfg['minimum_separation_m']and('usv'not in name or p['maximum_distance_upper_m']<=cfg['support_range_m'])for name,p in pairs.items())
    naive_summary={'all_constraints_certified':passes(naive_pairs),'pairs':naive_pairs,'initial_poses_preserved':True,'strategy':'All A6 individual curves start at0, each holds after finishing; no USV support redesign.'}
    (args.output_dir/'naive_baseline.json').write_text(json.dumps(naive_summary,indent=2)+'\n')
    clocks=None;departure_trials=[]
    for delay in cfg['uuv2_departure_candidates_s']:
        first=naive['uuv_1'];second=CurvedClock(x['platforms']['uuv_2']['pieces'],x['platforms']['uuv_2']['profile'],delay);end=max(first.finish_s,second.finish_s)
        pair=certify(first,second,end,f'uuv_departure_{delay:g}');departure_trials.append({'uuv2_departure_s':delay,'pair':pair})
        if pair['converged']and pair['minimum_distance_lower_m']>=cfg['minimum_separation_m']:clocks={'uuv_1':first,'uuv_2':second};break
    if clocks is None:raise ValueError('No certified UUV departure pair within fixed candidate budget')
    polylines=[];sagitta=0.
    for name in ('uuv_1','uuv_2'):
        xy,error=sample_enclosure([Piece(**p)for p in x['platforms'][name]['pieces']],cfg['maximum_arc_sagitta_m']);polylines.append(np.asarray(xy));sagitta=max(sagitta,error)
    hull=np.asarray(convex_hull(np.vstack(polylines).tolist()));axis=np.arange(cfg['station_grid_step_m']/2,5120,cfg['station_grid_step_m']);xx,yy=np.meshgrid(axis,axis);points=np.column_stack((xx.ravel(),yy.ravel()))
    worst=np.max(np.linalg.norm(points[:,None,:]-hull[None,:,:],axis=2),axis=1)+sagitta;keep=worst<=cfg['support_range_m']-cfg['station_range_reserve_m'];points,worst=points[keep],worst[keep]
    with NavigationGeometry(shapes['navigation'],shapes['hazards'])as n:
        valid=np.asarray([n.covers({'type':'Point','coordinates':p.tolist()})for p in points]);points,worst=points[valid],worst[valid]
        minimum=np.full(len(points),np.inf)
        for xy in polylines:
            for a,b in zip(xy,xy[1:]):
                delta=b-a;length2=delta@delta
                if length2==0:continue
                fraction=np.clip((points-a)@delta/length2,0,1);minimum=np.minimum(minimum,np.linalg.norm(points-a-fraction[:,None]*delta,axis=1))
        allowed=np.flatnonzero(minimum-sagitta>=cfg['station_route_clearance_m']);initial_pose=x['platforms']['usv']['pieces'][0]['pose']
        ordered=sorted(allowed,key=lambda i:(worst[i],-minimum[i],math.dist(initial_pose[:2],points[i]),points[i][0],points[i][1]))
        station_records=[{'position_m':points[i].tolist(),'worst_entire_route_range_upper_m':float(worst[i]),'entire_uuv_route_clearance_lower_m':float(minimum[i]-sagitta)}for i in ordered]
        (args.output_dir/'station_candidates.json').write_text(json.dumps({'inside_navigation_and_range_count':len(points),'all_geometry_clearance_pass_count':len(allowed),'ranked_candidates':station_records},indent=2)+'\n')
        attempts=[];chosen=None
        for station in station_records[:cfg['maximum_station_candidates']]:
            options=[]
            for yaw in (initial_pose[2],0.,math.pi/2,math.pi,-math.pi/2):
                for candidate in candidates(initial_pose,(*station['position_m'],yaw),cfg['usv_minimum_turning_radius_m']):
                    xy,error=sample_enclosure(candidate['pieces'],cfg['maximum_arc_sagitta_m']);shape={'type':'LineString','coordinates':xy}
                    envelope=n.geos.operation('buffer',shape,distance=(error+1e-5)/math.cos(math.pi/120),quadrant_segments=30)
                    if not n.covers(envelope):continue
                    clearance=n.distance(shape,to_hazards=True)-error
                    if clearance<50:continue
                    options.append({**candidate,'minimum_hazard_clearance_lower_m':clearance,'goal_heading_rad':yaw});break
            options.sort(key=lambda p:p['length_m'])
            for option_index,option in enumerate(options):
                profile=stopping_profile(option['pieces'],cfg['usv_cruise_mps'],cfg['tangential_accel_mps2'],cfg['lateral_accel_mps2'])
                for delay in cfg['usv_departure_candidates_s']:
                    surface=CurvedClock(option['pieces'],profile,delay);end=max(surface.finish_s,*[c.finish_s for c in clocks.values()]);pairs={'uuv_1__uuv_2':certify(clocks['uuv_1'],clocks['uuv_2'],end,f'candidate_{len(attempts)}_uuv_pair')}
                    for name in ('uuv_1','uuv_2'):pairs['usv__'+name]=certify(surface,clocks[name],end,f'candidate_{len(attempts)}_usv__{name}')
                    attempts.append({'station':station,'option_index':option_index,'usv_departure_s':delay,'all_constraints_certified':passes(pairs),'pairs':pairs,
                                     'usv_motion':{'pieces':[asdict(p)for p in option['pieces']],'profile':profile,'departure_s':delay}})
                    if passes(pairs):chosen=(station,option,profile,surface,end,pairs);break
                if chosen:break
            if chosen:break
        if chosen is None:raise ValueError('No certified support transfer/station within fixed search budget; partial evidence retained')
    station,option,profile,surface,end,pairs=chosen;clocks['usv']=surface;platforms={}
    for name,c in clocks.items():
        body=x['platforms'][name]if name!='usv'else{'pieces':[asdict(p)for p in option['pieces']],'profile':profile,'length_m':option['length_m']}
        holds=[]
        if c.departure_s>0:holds.append({'t0_s':0.,'t1_s':c.departure_s,'pose':list(c.initial_pose),'kind':'predeparture_hold','scanning':False})
        if c.finish_s<end:holds.append({'t0_s':c.finish_s,'t1_s':end,'pose':list(c.final_pose),'kind':'completed_hold','scanning':False})
        platforms[name]={'pieces':body['pieces'],'profile':body['profile'],'length_m':body['length_m'],'departure_s':c.departure_s,'motion_finish_s':c.finish_s,'initial_pose':list(c.initial_pose),'final_pose':list(c.final_pose),'holds':holds,'source_task_ids':body.get('task_ids',[])}
    windows=[]
    for name in ('uuv_1','uuv_2'):
        p=x['platforms'][name];offset=0
        for index,leg in enumerate(p['legs']):
            count=len(leg['pieces'])
            if leg['scanning']:
                a=p['profile']['segments'][offset]['t0_s'];b=p['profile']['segments'][offset+count-1]['t1_s'];windows.append({'task_id':leg['task_id'],'platform':name,'kind':'source_scan','source_leg_index':index,'local_start_s':a,'local_end_s':b,'global_start_s':a+clocks[name].departure_s,'global_end_s':b+clocks[name].departure_s})
            offset+=count
    for r in repair['selected_repairs']:
        name=r['platform'];windows.append({'task_id':r['task_id'],'platform':name,'kind':'repair_scan','source_leg_index':r['source_transit_leg_index'],'local_start_s':r['local_start_s'],'local_end_s':r['local_end_s'],'global_start_s':r['local_start_s']+clocks[name].departure_s,'global_end_s':r['local_end_s']+clocks[name].departure_s})
    windows.sort(key=lambda w:(w['platform'],w['global_start_s'],w['task_id']))
    with(args.output_dir/'global_scan_schedule.csv').open('w',newline='')as f:
        writer=csv.DictWriter(f,fieldnames=list(windows[0]));writer.writeheader();writer.writerows(windows)
    times={0.,end,*np.arange(0,end,cfg['trace_step_s']).tolist(),*[t for c in clocks.values()for t in c.phase_times if 0<=t<=end],*[w[k]for w in windows for k in('global_start_s','global_end_s')]}
    with(args.output_dir/'common_clock_trace.csv').open('w',newline='')as f:
        fields=['time_s','platform','east_m','north_m','heading_rad','speed_mps','mode','scanning','active_task_id'];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for t in sorted(times):
            for name in ('uuv_1','uuv_2','usv'):
                state=clocks[name].state(t);active=next((w['task_id']for w in windows if w['platform']==name and w['global_start_s']<=t<w['global_end_s']),'')
                writer.writerow({'time_s':t,'platform':name,'east_m':state['position_m'][0],'north_m':state['position_m'][1],'heading_rad':state['heading_rad'],'speed_mps':state['speed_mps'],'mode':state['mode'],'scanning':bool(active),'active_task_id':active})
    result={'stage':'A7','status':'curved_common_clock_certified','mission_end_s':end,'coverage_completion_s':max(clocks[n].finish_s for n in('uuv_1','uuv_2')),'platforms':platforms,'scan_windows':windows,'source_scan_count':86,'repair_scan_count':len(repair['selected_repairs']),
            'support_station':station,'usv_transfer_method':option['word'],'usv_transfer_hazard_clearance_lower_m':option['minimum_hazard_clearance_lower_m'],'distance_pairs':pairs,'uuv_departure_trials':departure_trials,'support_attempts':attempts,
            'coverage_lower_ratio_unchanged':repair['coverage_lower_ratio'],'coverage_target_sha256':cfg['target_geometry_sha256'],'all_initial_poses_preserved':True,'naive_A6_parallel_start_certified':naive_summary['all_constraints_certified'],
            'minimum_separation_m':cfg['minimum_separation_m'],'support_range_m':cfg['support_range_m'],'motion_constraints_scope':'Individual2D forward curvature-limited stop-to-stop kinematics, not full vehicle dynamics.',
            'support_scope':'Horizontal geometric range only, not acoustic communication or depth verification. Station holding assumes ideal point station-keeping. Deployment/recovery outside task clock.',
            'strategy':'UUV A6 curves/profiles unchanged; USV same initial pose, safe forward transfer to finite-grid station, then hold. No optimality claim.','total_computation_seconds':time.perf_counter()-started}
    (args.output_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    code=[Path(__file__),ROOT/'src/berthing_planner/curved_timing.py',ROOT/'src/berthing_planner/curves.py',ROOT/'src/berthing_planner/dubins.py',ROOT/'src/berthing_planner/motion_feasibility.py',ROOT/'scripts/navigation_geometry.py',ROOT/'scripts/chart_geometry.py',ROOT/'scripts/run_motion_feasibility.py']
    provenance={'input_sha256':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in files},'code_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in code},'result_sha256':hashlib.sha256((args.output_dir/'result.json').read_bytes()).hexdigest(),
                'derived_files_sha256':{str(p.relative_to(args.output_dir)):hashlib.sha256(p.read_bytes()).hexdigest()for p in args.output_dir.rglob('*')if p.is_file()and p.name not in('result.json','config.json')}}
    (args.output_dir/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n');print(json.dumps({'mission_end_s':end,'station':station,'pairs':pairs,'elapsed_s':result['total_computation_seconds']},indent=2))


if __name__=='__main__':main()
