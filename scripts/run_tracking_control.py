"""Simulate actual feedback-controlled ZOH trajectories on frozen A9 references."""
import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curved_timing import CurvedClock
from berthing_planner.scheduled_timing import ScheduledCurvedClock
from berthing_planner.tracking import feedback_command,plant_step
from tracking_geometry import evaluate_geometry


def canonical(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()


def reference_state(clock,t):
    state=clock.state(t);v=state['speed_mps'];yaw=state['heading_rad'];base=clock
    if isinstance(clock,ScheduledCurvedClock):
        index=state['leg_index']
        if index is None:return (*state['position_m'],yaw,0.,0.,0.,0.,0.,0.)
        base=clock.legs[index]
    if t<base.departure_s or t>=base.finish_s:
        return (*state['position_m'],yaw,0.,0.,0.,0.,0.,0.)
    index=state['piece_index'];piece=base.pieces[index];segment=base.segments[index]
    local=t-base.departure_s-segment['t0_s'];ta=segment['ramp_duration_s'];tc=segment['cruise_duration_s']
    tangential=segment['accel_mps2']if local<ta else 0. if local<ta+tc else-segment['accel_mps2']
    k=piece.curvature;c,s=math.cos(yaw),math.sin(yaw)
    return (*state['position_m'],yaw,v*c,v*s,k*v,tangential*c-k*v*v*s,tangential*s+k*v*v*c,k*tangential)


def pair_bounds(times,states,controls):
    dt=np.diff(times);columns=[];summaries={};names=('usv','uuv_1','uuv_2')
    for i,j in ((0,1),(0,2),(1,2)):
        r=states[:,i,:2]-states[:,j,:2];delta=np.diff(r,axis=0);squared=np.sum(delta*delta,axis=1)
        fraction=np.divide(-np.sum(r[:-1]*delta,axis=1),squared,out=np.zeros(len(dt)),where=squared>0);fraction=np.clip(fraction,0,1)
        centre=r[:-1]+fraction[:,None]*delta;chord_min=np.linalg.norm(centre,axis=1)
        accel=np.linalg.norm(controls[:,i,:2]-controls[:,j,:2],axis=1);pad=accel*dt**2/8+1e-6
        low=np.maximum(0,chord_min-pad);high=np.maximum(np.linalg.norm(r[:-1],axis=1),np.linalg.norm(r[1:],axis=1))+pad
        midpoint=(r[:-1]+r[1:])/2-(controls[:,i,:2]-controls[:,j,:2])*dt[:,None]**2/8
        witness_min=float(min(np.linalg.norm(r,axis=1).min(),np.linalg.norm(midpoint,axis=1).min()));witness_max=float(max(np.linalg.norm(r,axis=1).max(),np.linalg.norm(midpoint,axis=1).max()))
        label=names[i]+'__'+names[j];summaries[label]={'minimum_distance_lower_m':float(low.min()),'maximum_distance_upper_m':float(high.max()),'minimum_distance_witness_m':witness_min,'maximum_distance_witness_m':witness_max}
        columns.extend((low,high))
    return summaries,np.column_stack(columns)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a9-run-dir','a9-audit-json','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/tracking_control_a11.json');parser.add_argument('--case',default='nominal');parser.add_argument('--step-s',type=float)
    args=parser.parse_args();started=time.perf_counter();cfg=json.loads(args.config.read_text());scenario=cfg['cases'][args.case];step=cfg['step_s']if args.step_s is None else args.step_s
    if not math.isfinite(step)or not 0<step<=1:raise ValueError('Control period must be finite in (0,1] seconds')
    sourcefile=args.a9_run_dir/'result.json';source=json.loads(sourcefile.read_text());audit=json.loads(args.a9_audit_json.read_text())
    if canonical({k:v for k,v in source.items()if k!='total_computation_seconds'})!=cfg['source_a9_scientific_sha256']:raise ValueError('Frozen A9 science changed')
    if not audit['passed']or audit['source_result_sha256']!=hashlib.sha256(sourcefile.read_bytes()).hexdigest():raise ValueError('Matching passed A9 audit required')
    files=[sourcefile,args.a9_audit_json,args.config];shapes={}
    for name,key,directory in [('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir),('navigation','navigation_sha256',args.navigation_dir)]:
        f=directory/(name+'.geojson');files.append(f)
        if hashlib.sha256(f.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Frozen chart changed: '+name)
        shapes[name]=json.loads(f.read_text())['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False);(args.output_dir/'config.json').write_bytes(args.config.read_bytes())
    write=lambda name,value:(args.output_dir/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    clocks={'usv':ScheduledCurvedClock(source['mobile_usv']['legs'],source['mobile_usv']['initial_pose']),**{n:CurvedClock(p['pieces'],p['profile'],p['departure_s'])for n,p in source['uuv_platforms'].items()}}
    end=source['mission_end_s'];names=tuple(clocks);times=np.asarray(sorted({0.,end,*np.arange(0,end,step).tolist(),*[t for c in clocks.values()for t in c.phase_times if 0<t<end],*[w[k]for w in source['scan_windows']for k in ('global_start_s','global_end_s')]}))
    # Near-coincident phase times are kept. No finite differencing of reference velocities is used.
    reference=np.asarray([[reference_state(c,float(t))for c in clocks.values()]for t in times]);states=np.empty((len(times),3,6));states[0]=reference[0,:,:6]
    states[0,:,:2]+=np.asarray(scenario['initial_offset_m']);states[0,:,2]+=math.radians(scenario['initial_heading_error_deg']);controls=np.empty((len(times)-1,3,3));integrals=[(0.,0.)]*3
    drift=np.asarray(scenario['drift_mps']);bias=np.asarray(scenario['position_bias_m']);noise=scenario['position_noise_amplitude_m'];heading_bias=math.radians(scenario['heading_bias_deg'])
    for i,h in enumerate(np.diff(times)):
        for j,name in enumerate(names):
            state=states[i,j];sensed=state.copy();sensed[:2]+=bias[j]+noise*np.array([math.sin(times[i]*.7+j),math.cos(times[i]*.7+j)]);sensed[2]+=heading_bias;sensed[3:5]+=drift[j]
            mid=reference_state(clocks[name],float(times[i]+h/2));cmd,integrals[j]=feedback_command(state,sensed,reference[i,j,:6],mid[6:9],integrals[j],float(h),cfg['limits'][name],cfg['gains'])
            controls[i,j]=cmd;states[i+1,j]=plant_step(state,cmd,drift[j],float(h))
    np.savez_compressed(args.output_dir/'trajectory.npz',times=times,states=states,controls=controls,reference=reference)
    print('Integrated',len(times)-1,'actual control intervals',flush=True)
    pairs,certificates=pair_bounds(times,states,controls);np.savez_compressed(args.output_dir/'distance_intervals.npz',bounds=certificates)
    geometry,covered,missed=evaluate_geometry(times,states,controls,source['scan_windows'],shapes,cfg);write('geometry_enclosures.json',{'paths':geometry.pop('paths'),'scans':geometry.pop('scans')});write('actual_covered_lower.geojson',{'type':'FeatureCollection','features':[{'type':'Feature','properties':{},'geometry':covered}]});write('actual_missed_upper.geojson',{'type':'FeatureCollection','features':[{'type':'Feature','properties':{},'geometry':missed}]})
    metrics={}
    for j,name in enumerate(names):
        error=np.linalg.norm(states[:,j,:2]-reference[:,j,:2],axis=1);heading=np.abs((states[:,j,2]-reference[:,j,2]+math.pi)%(2*math.pi)-math.pi);speed=np.linalg.norm(states[:,j,3:5]+drift[j]-reference[:,j,3:5],axis=1)
        metrics[name]={'maximum_position_error_m':float(error.max()),'time_weighted_position_rmse_m':float(math.sqrt(np.trapezoid(error**2,times)/end)),'maximum_heading_error_rad':float(heading.max()),'maximum_ground_velocity_error_mps':float(speed.max()),'maximum_command_acceleration_mps2':float(np.linalg.norm(controls[:,j,:2],axis=1).max()),'maximum_yaw_acceleration_radps2':float(np.abs(controls[:,j,2]).max()),'maximum_water_relative_speed_mps':float(np.linalg.norm(states[:,j,3:5],axis=1).max())}
    safe=all(v['minimum_distance_lower_m']>=cfg['minimum_separation_m']for v in pairs.values())and max(v['maximum_distance_upper_m']for k,v in pairs.items()if k.startswith('usv__'))<=cfg['support_range_m']
    result={'stage':cfg['stage'],'case':args.case,'scenario':scenario,'effective_step_s':step,'mission_end_s':end,'reference_scan_count':len(source['scan_windows']),'source_a9_scientific_sha256':cfg['source_a9_scientific_sha256'],'metrics':metrics,'distance_pairs':pairs,'geometry':geometry,'mission_constraints_passed':bool(safe and geometry['all_geometry_passed']and geometry['coverage_lower_ratio']>=cfg['minimum_coverage_ratio']),'scope':cfg['scope'],'total_computation_seconds':time.perf_counter()-started};write('result.json',result)
    code=[Path(__file__),ROOT/'src/berthing_planner/tracking.py',ROOT/'src/berthing_planner/curved_timing.py',ROOT/'src/berthing_planner/scheduled_timing.py',ROOT/'scripts/tracking_geometry.py',ROOT/'scripts/chart_geometry.py',ROOT/'scripts/navigation_geometry.py']
    write('provenance.json',{'input_sha256':{str(f.resolve()):hashlib.sha256(f.read_bytes()).hexdigest()for f in files},'code_sha256':{str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()for f in code},'result_sha256':hashlib.sha256((args.output_dir/'result.json').read_bytes()).hexdigest(),'derived_sha256':{str(f.relative_to(args.output_dir)):hashlib.sha256(f.read_bytes()).hexdigest()for f in args.output_dir.iterdir()if f.is_file()and f.name not in('result.json','provenance.json')}})
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
