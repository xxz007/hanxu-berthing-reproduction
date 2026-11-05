"""A9 finite look-ahead and transfer-cost improvement with frozen A8 conditions."""
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
from berthing_planner.curved_timing import CurvedClock,continuous_distance_bounds
from berthing_planner.scheduled_timing import ScheduledCurvedClock,WindowClock,classify_support_range
from berthing_planner.dubins import candidates
from berthing_planner.motion_feasibility import stopping_profile
from .navigation_geometry import NavigationGeometry
from .curve_geometry import sample_enclosure


def canonical(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def run(*, seed_dir, prepared_dir, navigation_dir, config, output_dir):
    started=time.perf_counter();cfg=json.loads(config.read_text());sourcefile=seed_dir/'result.json';x=json.loads(sourcefile.read_text())
    auditfile=seed_dir/'audit.json';audit=json.loads(auditfile.read_text())
    if not audit['passed']or audit['source_result_sha256']!=hashlib.sha256(sourcefile.read_bytes()).hexdigest():raise ValueError('Audited joint scan schedule required')
    shapes={};inputs=[config,sourcefile,auditfile]
    for name,key,directory in [('searchable','target_geometry_sha256',prepared_dir),('hazards','hazards_geometry_sha256',prepared_dir),('navigation','navigation_sha256',navigation_dir)]:
        path=directory/(name+'.geojson');inputs.append(path)
        if hashlib.sha256(path.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Frozen chart geometry changed: '+name)
        shapes[name]=json.loads(path.read_text())['features'][0]['geometry']
    output_dir.mkdir(parents=True,exist_ok=False);(output_dir/'certificates').mkdir();(output_dir/'config.json').write_bytes(config.read_bytes())
    write=lambda path,value:(output_dir/path).write_text(json.dumps(value,indent=2)+'\n')
    write('certificates/uuv_pair.json',json.loads((seed_dir/'uuv_pair.json').read_text()))
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
        windows=[(float(a),min(float(a)+cfg['window_duration_s'],end))for a in np.arange(0,end,cfg['window_duration_s'])]
        # Each sample only eliminates stations or ranks finite candidates; it cannot accept a route.
        maxima=[];minima=[]
        for a,b in windows:
            times=sorted({a,b,*np.arange(a,b,cfg['candidate_time_step_s']).tolist(),*[t for c in uuv.values()for t in c.phase_times if a<=t<=b]})
            points=np.asarray([c.state(t)['position_m']for c in uuv.values()for t in times])
            distances=np.linalg.norm(grid[:,None,:]-points[None,:,:],axis=2)
            maxima.append(distances.max(axis=1));minima.append(distances.min(axis=1))
        maxima=np.asarray(maxima);minima=np.asarray(minima)
        for index,(a,b) in enumerate(windows):
            a=float(a);b=min(a+cfg['window_duration_s'],end);prior=ScheduledCurvedClock(legs,initial);hold_pairs,hold_reports,hold_pass=certify(prior,a,b,'hold');attempts=[];selected=None
            if hold_pass:selected=('hold',None,hold_pairs,hold_reports)
            else:
                pose=prior.state(a);start=(*pose['position_m'],pose['heading_rad'])
                # Longest sampled feasible hold horizon first; ties prefer less travel, then coordinates.
                ranked=[];seen=set()
                worst=np.maximum.accumulate(maxima[index:],axis=0);closest=np.minimum.accumulate(minima[index:],axis=0)
                for offset in range(len(windows)-index-1,-1,-1):
                    valid=np.flatnonzero((worst[offset]<=cfg['mobile_design_range_m']-cfg['station_sample_range_reserve_m'])&(closest[offset]>=cfg['station_sample_clearance_m']))
                    nearest=sorted(valid,key=lambda i:(math.dist(start[:2],grid[i]),grid[i][0],grid[i][1]))[:cfg['maximum_station_candidates_per_horizon']]
                    for i in nearest:
                        if int(i)not in seen:ranked.append((int(i),windows[index+offset][1]));seen.add(int(i))
                order=[i for i,_ in ranked];horizons=dict(ranked)
                for rank,i in enumerate(order):
                    target=grid[i];yaw=math.atan2(target[1]-start[1],target[0]-start[0]);options=[]
                    for heading in dict.fromkeys((yaw,start[2],0.,math.pi/2,math.pi,-math.pi/2)):
                        for option in candidates(start,(*target,heading),cfg['usv_radius_m']):
                            profile=stopping_profile(option['pieces'],cfg['usv_cruise_mps'],cfg['tangential_accel_mps2'],cfg['lateral_accel_mps2'])
                            if profile['duration_s']>=b-a:continue
                            options.append((profile['duration_s'],option['word'],heading,option,profile))
                    options.sort(key=lambda v:(v[3]['length_m'],v[0],v[1],v[2]))
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
                        if passed:
                            body['sample_lookahead_end_s']=horizons[i];body['target_pose']=[*target,option['pieces'][-1].evaluate(1)[2]]
                            selected=('transfer',body,pairs,reports);break
                    if selected:break
            decision={'index':index,'t0_s':a,'t1_s':b,'hold_control_pairs':hold_pairs,'attempts':attempts}
            for name,report in hold_reports.items():write(f'certificates/window_{index:02d}_hold_{name}.json',report)
            if selected is None:
                decision['action']='finite_search_unresolved';decisions.append(decision);write('partial_plan.json',{'legs':legs,'decisions':decisions,'necessary_conditions':necessary});raise ValueError(f'Window{index} no certified candidate in finite search; not a global infeasibility proof')
            action,body,pairs,reports=selected;decision.update(action=action,pairs=pairs)
            if body:legs.append(body);decision['leg_index']=len(legs)-1;minimum_clearance=min(minimum_clearance,body['minimum_hazard_clearance_lower_m'])
            for name,report in reports.items():write(f'certificates/window_{index:02d}_selected_{name}.json',report)
            decisions.append(decision);write('progress.json',{'completed_windows':len(decisions),'legs':len(legs),'latest_window':decision});print(f'window{index:02d}: {action}; legs={len(legs)}',flush=True)
    mobile=ScheduledCurvedClock(legs,initial);pairs={'uuv_1__uuv_2':x['distance_pairs']['uuv_1__uuv_2']}
    for name,c in uuv.items():
        report=continuous_distance_bounds(mobile,c,end,**settings);rel='certificates/mission_usv__'+name+'.json';write(rel,report)
        pairs['usv__'+name]={k:v for k,v in report.items()if k!='partition'}|{'certificate_file':rel}
    # All three pairs, not just the finite-window candidate samples, must pass.
    if not all(p['converged']and p['minimum_distance_lower_m']>=cfg['minimum_separation_m']and(not n.startswith('usv__')or p['maximum_distance_upper_m']<=cfg['mobile_design_range_m'])for n,p in pairs.items()):
        raise ValueError('Whole-mission support/separation was not certified')
    holds=[];last=0.;pose=initial
    for c in mobile.legs:
        if c.departure_s>last:holds.append({'t0_s':last,'t1_s':c.departure_s,'pose':list(pose),'scanning':False})
        last=c.finish_s;pose=c.final_pose
    if last<end:holds.append({'t0_s':last,'t1_s':end,'pose':list(pose),'scanning':False})
    result={'kind':'cooperative_reference','mission_end_s':end,'uuv_platforms':{n:x['platforms'][n]for n in uuv},'scan_windows':x['scan_windows'],'coverage_lower_ratio_unchanged':x['coverage_lower_ratio_unchanged'],'source_original_scan_count':86,'source_repair_scan_count':3,'mobile_usv':{'initial_pose':initial,'legs':legs,'holds':holds,'length_m':mobile.length_m,'motion_finish_s':mobile.finish_s},'window_decisions':decisions,'necessary_conditions':necessary,'distance_pairs':pairs,'mobile_design_range_m':cfg['mobile_design_range_m'],'minimum_hazard_clearance_lower_m':minimum_clearance,'scope':cfg['policy'],'total_computation_seconds':time.perf_counter()-started}
    write('result.json',result)
    write('provenance.json',{'input_sha256':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in inputs},'result_sha256':hashlib.sha256((output_dir/'result.json').read_bytes()).hexdigest(),'derived_sha256':{str(p.relative_to(output_dir)):hashlib.sha256(p.read_bytes()).hexdigest()for p in sorted(output_dir.rglob('*'))if p.is_file()and p.name not in('result.json','provenance.json','progress.json')}})
    print('Reference certified:',len(legs),'USV transfers;',mobile.length_m,'m')
