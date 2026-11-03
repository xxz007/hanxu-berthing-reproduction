"""A8 frozen-UUV support-range sweep and finite-window mobile USV planning."""
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
from berthing_planner.curved_timing import CurvedClock,continuous_distance_bounds
from berthing_planner.scheduled_timing import ScheduledCurvedClock,WindowClock,classify_support_range
from berthing_planner.dubins import candidates
from berthing_planner.motion_feasibility import stopping_profile
from navigation_geometry import NavigationGeometry
from run_motion_feasibility import sample_enclosure


def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a7-run-dir','a7-audit-json','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/support_sensitivity_a8.json');args=parser.parse_args();started=time.perf_counter();cfg=json.loads(args.config.read_text())
    sourcefile=args.a7_run_dir/'result.json';x=json.loads(sourcefile.read_text());audit=json.loads(args.a7_audit_json.read_text());digest=hashlib.sha256(sourcefile.read_bytes()).hexdigest()
    if canonical({k:v for k,v in x.items()if k!='total_computation_seconds'})!=cfg['source_a7_scientific_sha256']:raise ValueError('Frozen A7 science changed')
    if not audit['passed']or audit['source_result_sha256']!=digest:raise ValueError('Matching passed A7 audit required')
    shapes={};inputs=[args.config,sourcefile,args.a7_audit_json]
    for name,key,directory in [('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir),('navigation','navigation_sha256',args.navigation_dir)]:
        path=directory/(name+'.geojson');inputs.append(path)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Frozen chart geometry changed: '+name)
        shapes[name]=json.loads(path.read_text())['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False);(args.output_dir/'certificates').mkdir();(args.output_dir/'config.json').write_bytes(args.config.read_bytes())
    write=lambda path,value:(args.output_dir/path).write_text(json.dumps(value,indent=2)+'\n')
    settings={'tolerance_m':cfg['distance_tolerance_m'],'numerical_margin_m':cfg['numerical_margin_m'],'max_refinements':cfg['max_distance_refinements']}
    uuv={n:CurvedClock(p['pieces'],p['profile'],p['departure_s'])for n,p in x['platforms'].items()if n!='usv'};end=x['mission_end_s'];initial=x['platforms']['usv']['initial_pose'];legs=[];decisions=[];minimum_clearance=math.inf
    initial_distances={n:math.dist(initial[:2],c.state(0)['position_m'])for n,c in uuv.items()};initial_required=max(initial_distances.values())
    pair=x['distance_pairs']['uuv_1__uuv_2'];necessary={'initial_usv_uuv_distances_m':initial_distances,'initial_required_support_range_m':initial_required,'uuv_pair_half_maximum_witness_m':pair['maximum_distance_witness_m']/2,'uuv_pair_witness_time_s':pair['maximum_witness_time_s'],'scope':'Initial distances are necessary for any USV policy with same fixed initial poses and support required at t=0. Pair half-distance is another necessary bound at its witnessed time, independent of USV speed or obstacles.'}
    def certify(clock,a,b,label):
        pairs={};records={}
        for name,c in uuv.items():
            r=continuous_distance_bounds(WindowClock(clock,a,b),WindowClock(c,a,b),b-a,**settings);records[name]=r;pairs['usv__'+name]={k:v for k,v in r.items()if k!='partition'}
        passed=all(p['converged']and p['minimum_distance_lower_m']>=cfg['minimum_separation_m']and p['maximum_distance_upper_m']<=cfg['mobile_design_range_m']for p in pairs.values())
        return pairs,records,passed
    axis=lambda bounds:np.arange(bounds[0],bounds[1]+cfg['candidate_grid_step_m']/2,cfg['candidate_grid_step_m'])
    xx,yy=np.meshgrid(axis(cfg['candidate_x_bounds_m']),axis(cfg['candidate_y_bounds_m']));grid=np.column_stack((xx.ravel(),yy.ravel()))
    with NavigationGeometry(shapes['navigation'],shapes['hazards'])as nav:
        grid=grid[[nav.covers({'type':'Point','coordinates':p.tolist()})for p in grid]]
        for index,a in enumerate(np.arange(0,end,cfg['window_duration_s'])):
            a=float(a);b=min(a+cfg['window_duration_s'],end);prior=ScheduledCurvedClock(legs,initial);hold_pairs,hold_reports,hold_pass=certify(prior,a,b,'hold');attempts=[];selected=None
            if hold_pass:selected=('hold',None,hold_pairs,hold_reports)
            else:
                times=sorted({a,b,*np.arange(a,b,cfg['candidate_time_step_s']).tolist()});future=np.asarray([c.state(t)['position_m']for c in uuv.values()for t in times]);distances=np.linalg.norm(grid[:,None,:]-future[None,:,:],axis=2);worst=distances.max(axis=1);closest=distances.min(axis=1)
                valid=np.flatnonzero((worst<=cfg['mobile_design_range_m']-cfg['station_sample_range_reserve_m'])&(closest>=cfg['station_sample_clearance_m']));pose=prior.state(a);start=(*pose['position_m'],pose['heading_rad'])
                order=sorted(valid,key=lambda i:(worst[i],math.dist(start[:2],grid[i]),grid[i][0],grid[i][1]))[:cfg['maximum_station_candidates_per_window']]
                for rank,i in enumerate(order):
                    target=grid[i];yaw=math.atan2(target[1]-start[1],target[0]-start[0]);options=[]
                    for heading in dict.fromkeys((yaw,start[2],0.,math.pi/2,math.pi,-math.pi/2)):
                        for option in candidates(start,(*target,heading),cfg['usv_radius_m']):
                            profile=stopping_profile(option['pieces'],cfg['usv_cruise_mps'],cfg['tangential_accel_mps2'],cfg['lateral_accel_mps2'])
                            if profile['duration_s']>=b-a:continue
                            options.append((profile['duration_s'],option['word'],heading,option,profile))
                    options.sort(key=lambda v:(v[0],v[1],v[2]))
                    for _,_,_,option,profile in options:
                        xy,error=sample_enclosure(option['pieces'],cfg['maximum_arc_sagitta_m']);shape={'type':'LineString','coordinates':xy};envelope=nav.geos.operation('buffer',shape,distance=(error+1e-5)/math.cos(math.pi/120),quadrant_segments=30)
                        if not nav.covers(envelope):attempts.append({'candidate_rank':rank,'word':option['word'],'reason':'navigation_enclosure'});continue
                        clearance=nav.distance(shape,to_hazards=True)-error
                        if clearance<cfg['hazard_margin_m']:attempts.append({'candidate_rank':rank,'word':option['word'],'reason':'hazard_margin'});continue
                        body={'departure_s':a,'pieces':[asdict(p)for p in option['pieces']],'profile':profile,'minimum_hazard_clearance_lower_m':clearance,'word':option['word']};trial=ScheduledCurvedClock([*legs,body],initial)
                        # Saved samples reject obvious poor candidates only; all passes need whole-window bounds.
                        probe=sorted({a,b,*np.arange(a,b,30).tolist()});bad=any(math.dist(trial.state(t)['position_m'],c.state(t)['position_m'])>cfg['mobile_design_range_m']or math.dist(trial.state(t)['position_m'],c.state(t)['position_m'])<cfg['minimum_separation_m']for t in probe for c in uuv.values())
                        if bad:attempts.append({'candidate_rank':rank,'word':option['word'],'reason':'sample_witness_rejection'});continue
                        pairs,reports,passed=certify(trial,a,b,'move');attempts.append({'candidate_rank':rank,'word':option['word'],'reason':'certified_pass'if passed else'interval_not_passed','pairs':pairs})
                        if passed:selected=('transfer',body,pairs,reports);break
                    if selected:break
            decision={'index':index,'t0_s':a,'t1_s':b,'hold_control_pairs':hold_pairs,'attempts':attempts}
            for name,report in hold_reports.items():write(f'certificates/window_{index:02d}_hold_{name}.json',report)
            if selected is None:
                decision['action']='finite_search_unresolved';decisions.append(decision);write('partial_plan.json',{'legs':legs,'decisions':decisions,'necessary_conditions':necessary});raise ValueError(f'Window{index} no certified candidate in finite search; not a global infeasibility proof')
            action,body,pairs,reports=selected;decision.update(action=action,pairs=pairs)
            if body:legs.append(body);decision['leg_index']=len(legs)-1;minimum_clearance=min(minimum_clearance,body['minimum_hazard_clearance_lower_m'])
            for name,report in reports.items():write(f'certificates/window_{index:02d}_selected_{name}.json',report)
            decisions.append(decision);write('progress.json',{'completed_windows':len(decisions),'legs':len(legs),'latest_window':decision});print(f'window{index:02d}: {action}; legs={len(legs)}',flush=True)
    mobile=ScheduledCurvedClock(legs,initial);fixed=CurvedClock(x['platforms']['usv']['pieces'],x['platforms']['usv']['profile'],x['platforms']['usv']['departure_s']);strategies={};all_reports={}
    for strategy,clock in [('a7_fixed_station',fixed),('scheduled_mobile',mobile)]:
        pairs={}
        for name,c in uuv.items():
            r=continuous_distance_bounds(clock,c,end,**settings);rel=f'certificates/{strategy}_{name}.json';write(rel,r);pairs['usv__'+name]={k:v for k,v in r.items()if k!='partition'}|{'certificate_file':rel}
        strategies[strategy]={'pairs':pairs,'usv_length_m':mobile.length_m if strategy=='scheduled_mobile'else x['platforms']['usv']['length_m'],'usv_motion_duration_s':sum(c.duration_s for c in mobile.legs)if strategy=='scheduled_mobile'else fixed.duration_s}
    sweep=[]
    for required in cfg['range_grid_m']:
        sweep.append({'support_range_m':required,**{s:classify_support_range(list(v['pairs'].values()),required,cfg['minimum_separation_m'],initial_required)for s,v in strategies.items()}})
    holds=[];last=0.;pose=initial
    for c in mobile.legs:
        if c.departure_s>last:holds.append({'t0_s':last,'t1_s':c.departure_s,'pose':list(pose),'scanning':False})
        last=c.finish_s;pose=c.final_pose
    if last<end:holds.append({'t0_s':last,'t1_s':end,'pose':list(pose),'scanning':False})
    result={'stage':'A8','mission_end_s':end,'all_A7_initial_poses_preserved':True,'uuv_platforms':{n:x['platforms'][n]for n in uuv},'scan_windows':x['scan_windows'],'coverage_lower_ratio_unchanged':x['coverage_lower_ratio_unchanged'],'source_original_scan_count':86,'source_repair_scan_count':3,'mobile_usv':{'initial_pose':initial,'legs':legs,'holds':holds,'length_m':mobile.length_m,'motion_finish_s':mobile.finish_s},'window_decisions':decisions,'necessary_conditions':necessary,'strategies':strategies,'range_sweep':sweep,'mobile_design_range_m':cfg['mobile_design_range_m'],'minimum_hazard_clearance_lower_m':minimum_clearance,'scope':cfg['policy'],'total_computation_seconds':time.perf_counter()-started};write('result.json',result)
    with(args.output_dir/'range_sweep.csv').open('w',newline='')as f:
        writer=csv.DictWriter(f,fieldnames=list(sweep[0]));writer.writeheader();writer.writerows(sweep)
    times=sorted({0.,end,*np.arange(0,end,cfg['trace_step_s']).tolist(),*[t for c in [mobile,*uuv.values()]for t in c.phase_times if 0<=t<=end],*[w[k]for w in x['scan_windows']for k in ('global_start_s','global_end_s')]})
    with(args.output_dir/'mobile_trace.csv').open('w',newline='')as f:
        writer=csv.DictWriter(f,fieldnames=['time_s','platform','east_m','north_m','heading_rad','speed_mps','mode','active_task_id']);writer.writeheader()
        for t in times:
            for name,c in {'usv':mobile,**uuv}.items():
                s=c.state(t);task=next((w['task_id']for w in x['scan_windows']if w['platform']==name and w['global_start_s']<=t<w['global_end_s']),'');writer.writerow({'time_s':t,'platform':name,'east_m':s['position_m'][0],'north_m':s['position_m'][1],'heading_rad':s['heading_rad'],'speed_mps':s['speed_mps'],'mode':s['mode'],'active_task_id':task})
    code=[Path(__file__),ROOT/'src/berthing_planner/scheduled_timing.py',ROOT/'src/berthing_planner/curved_timing.py',ROOT/'src/berthing_planner/dubins.py',ROOT/'src/berthing_planner/curves.py',ROOT/'src/berthing_planner/motion_feasibility.py',ROOT/'scripts/navigation_geometry.py',ROOT/'scripts/chart_geometry.py',ROOT/'scripts/run_motion_feasibility.py']
    write('provenance.json',{'input_sha256':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in inputs},'code_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in code},'result_sha256':hashlib.sha256((args.output_dir/'result.json').read_bytes()).hexdigest(),'derived_files_sha256':{str(p.relative_to(args.output_dir)):hashlib.sha256(p.read_bytes()).hexdigest()for p in args.output_dir.rglob('*')if p.is_file()and p.name not in('result.json','provenance.json','progress.json')}})
    print(json.dumps({'range_sweep':sweep,'mobile_legs':len(legs),'mobile_length_m':mobile.length_m,'elapsed_s':result['total_computation_seconds']},indent=2))


if __name__=='__main__':main()
