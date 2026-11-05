"""Independent trapezoidal plant identities, interval motion bounds and footprint audit.

Does not import the production controller, plant integrator or distance solver.
For constant acceleration, endpoint-averaged velocity integrates position exactly.
"""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from chart_geometry import Geos
from audit_curved_cooperation import IndependentClock
from audit_support_sensitivity import IndependentSchedule


def plant_residuals(times,states,controls,drift):
    dt=np.diff(times)[:,None,None]
    velocity=float(np.max(np.abs(np.diff(states[:,:,3:6],axis=0)-controls*dt)))
    translation=np.diff(states[:,:,:2],axis=0)-dt*(.5*(states[:-1,:,3:5]+states[1:,:,3:5])+drift)
    yaw=np.diff(states[:,:,2],axis=0)-np.diff(times)[:,None]*.5*(states[:-1,:,5]+states[1:,:,5])
    return {'velocity_integral_error':velocity,'position_integral_error_m':float(np.max(np.abs(translation))),'heading_integral_error_rad':float(np.max(np.abs(yaw)))}


def enclosure_error(xy,indices):
    if not indices or indices[0]!=0 or indices[-1]!=len(xy)-1 or any(b<=a for a,b in zip(indices,indices[1:])):raise ValueError('Simplification must partition the full path')
    error=0.
    for a,b in zip(indices,indices[1:]):
        delta=xy[b]-xy[a];norm=float(delta@delta);q=xy[a:b+1]
        u=np.clip(np.sum((q-xy[a])*delta,axis=1)/norm,0,1)if norm else np.zeros(len(q))
        error=max(error,float(np.max(np.linalg.norm(q-xy[a]-u[:,None]*delta,axis=1))))
    return error


def independent_reference(clock,t):
    # Separate circle-centre/phase-integral implementation inherited from A7.
    p,_,speed,yaw=clock.state(t);base=clock
    if isinstance(clock,IndependentSchedule):
        eligible=[c for c in clock.clocks if c.departure<=t]
        base=eligible[-1]if eligible else None
    if base is None or t<=base.departure or t>=base.finish:return (*p,yaw,0.,0.,0.)
    i=max(0,bisect_right(base.starts,t-base.departure)-1);k=base.pieces[i]['curvature']
    return (*p,yaw,speed*math.cos(yaw),speed*math.sin(yaw),k*speed)


def independent_feedforward(clock,t):
    p,_,speed,yaw=clock.state(t);base=clock
    if isinstance(clock,IndependentSchedule):
        eligible=[c for c in clock.clocks if c.departure<=t];base=eligible[-1]if eligible else None
    if base is None or t<base.departure or t>=base.finish:return np.zeros(3)
    i=max(0,bisect_right(base.starts,t-base.departure)-1);segment=base.profile['segments'][i];tau=t-base.departure-segment['t0_s'];ramp=segment['ramp_duration_s'];cruise=segment['cruise_duration_s'];k=base.pieces[i]['curvature']
    tangential=segment['accel_mps2']*(1 if tau<ramp else -1 if tau>=ramp+cruise else 0)
    return np.array([tangential*math.cos(yaw)-speed**2*k*math.sin(yaw),tangential*math.sin(yaw)+speed**2*k*math.cos(yaw),k*tangential])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','a9-run-dir','prepared-dir','navigation-dir','json-out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();read=lambda f:json.loads(f.read_text());digest=lambda f:hashlib.sha256(f.read_bytes()).hexdigest();cfg=read(args.run_dir/'config.json');x=read(args.run_dir/'result.json');proof=read(args.run_dir/'provenance.json');source=read(args.a9_run_dir/'result.json');errors=[]
    if digest(args.run_dir/'result.json')!=proof['result_sha256']:errors.append('result hash')
    for path,h in proof['input_sha256'].items():
        if digest(Path(path))!=h:errors.append('input hash')
    for path,h in proof['code_sha256'].items():
        if digest(ROOT/path)!=h:errors.append('code hash')
    for path,h in proof['derived_sha256'].items():
        if digest(args.run_dir/path)!=h:errors.append('derived hash')
    canon=lambda v:hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    if canon({k:v for k,v in source.items()if k!='total_computation_seconds'})!=cfg['source_a9_scientific_sha256']:errors.append('A9 science pin')
    # Engineering limits cannot silently redefine the inherited mission gates.
    if (cfg['minimum_separation_m'],cfg['support_range_m'],cfg['hazard_margin_m'],cfg['minimum_coverage_ratio'],cfg['sensor_radius_m'])!=(25,2300,50,.999,50):errors.append('mission gates changed')
    z=np.load(args.run_dir/'trajectory.npz');times=z['times'];states=z['states'];commands=z['controls'];reference=z['reference'];dt=np.diff(times);scenario=cfg['cases'][x['case']];drift=np.asarray(scenario['drift_mps']);names=('usv','uuv_1','uuv_2')
    if states.shape!=(len(times),3,6)or commands.shape!=(len(dt),3,3)or reference.shape!=(len(times),3,9):raise ValueError('Full three-platform trajectory required')
    if not all(np.isfinite(v).all()for v in (times,states,commands,reference))or np.any(dt<=0)or dt.max()>x['effective_step_s']+1e-7 or times[0]!=0 or times[-1]!=source['mission_end_s']:raise ValueError('Finite complete ordered time grid required')
    if x['scenario']!=scenario or x['reference_scan_count']!=89 or len(source['scan_windows'])!=89:errors.append('scenario or scan inheritance')
    initial=reference[0,:,:6].copy();initial[:,:2]+=np.asarray(scenario['initial_offset_m']);initial[:,2]+=math.radians(scenario['initial_heading_error_deg'])
    if np.max(np.abs(states[0]-initial))>1e-8:errors.append('declared initial offset')
    residual=plant_residuals(times,states,commands,drift)
    if max(residual.values())>1e-7:errors.append('independent plant integration')
    clocks={'usv':IndependentSchedule(source['mobile_usv']['legs'],source['mobile_usv']['initial_pose']),**{n:IndependentClock(p)for n,p in source['uuv_platforms'].items()}}
    max_reference=0.;controller_error=0.
    for j,name in enumerate(names):
        independent=np.asarray([independent_reference(clocks[name],float(t))for t in times]);difference=reference[:,j,:6]-independent;difference[:,2]=(difference[:,2]+math.pi)%(2*math.pi)-math.pi;max_reference=max(max_reference,float(np.max(np.abs(difference))))
        limits=cfg['limits'][name]
        if np.linalg.norm(commands[:,j,:2],axis=1).max()>limits['acceleration_mps2']+1e-8 or np.abs(commands[:,j,2]).max()>limits['yaw_acceleration_radps2']+1e-8 or np.linalg.norm(states[:,j,3:5],axis=1).max()>limits['speed_mps']+1e-8 or np.abs(states[:,j,5]).max()>limits['yaw_rate_radps']+1e-8:errors.append('control/speed saturation')
        e=np.linalg.norm(states[:,j,:2]-independent[:,:2],axis=1)
        if abs(float(e.max())-x['metrics'][name]['maximum_position_error_m'])>1e-7:errors.append('position error metric')
        yaw=np.abs((states[:,j,2]-independent[:,2]+math.pi)%(2*math.pi)-math.pi)
        if abs(float(yaw.max())-x['metrics'][name]['maximum_heading_error_rad'])>1e-7:errors.append('heading error metric')
        gain=cfg['gains'];accumulated=np.zeros(2)
        for i,h in enumerate(dt):
            noisy=np.array([math.sin(times[i]*.7+j),math.cos(times[i]*.7+j)])*scenario['position_noise_amplitude_m']
            error=independent[i,:2]-states[i,j,:2]-np.asarray(scenario['position_bias_m'][j])-noisy
            accumulated=np.clip(accumulated+h*error,-gain['integral_limit'],gain['integral_limit'])
            ff=independent_feedforward(clocks[name],float(times[i]+h/2))
            acceleration=ff[:2]+gain['position_p']*error+gain['velocity_d']*(independent[i,3:5]-states[i,j,3:5]-drift[j])+gain['position_i']*accumulated
            acceleration*=min(1.,limits['acceleration_mps2']/max(float(np.linalg.norm(acceleration)),1e-300))
            next_velocity=states[i,j,3:5]+acceleration*h;next_velocity*=min(1.,limits['speed_mps']/max(float(np.linalg.norm(next_velocity)),1e-300))
            alpha=ff[2]+gain['heading_p']*math.remainder(independent[i,2]-states[i,j,2]-math.radians(scenario['heading_bias_deg']),2*math.pi)+gain['yaw_rate_d']*(independent[i,5]-states[i,j,5])
            alpha=np.clip(alpha,-limits['yaw_acceleration_radps2'],limits['yaw_acceleration_radps2']);rate=np.clip(states[i,j,5]+h*alpha,-limits['yaw_rate_radps'],limits['yaw_rate_radps'])
            expected=np.r_[(next_velocity-states[i,j,3:5])/h,(rate-states[i,j,5])/h]
            # Extremely short coincident phase intervals amplify last-bit state
            # differences when dividing by h; compare their velocity impulses.
            controller_error=max(controller_error,float(np.max(np.abs((expected-commands[i,j])*min(1.,h)))))
    if max_reference>1e-6:errors.append('independent reference integration')
    if controller_error>1e-7:errors.append('independent feedback command replay')
    pairs={};witness_violation=False
    for i,j in ((0,1),(0,2),(1,2)):
        # Midpoint plus a bound on BOTH travelled lengths; independent of the
        # production chord/second-derivative certificate formula.
        mid=states[:-1,:,:2]+dt[:,None,None]*.5*(states[:-1,:,3:5]+drift)+commands[:,:,:2]*dt[:,None,None]**2/8
        distance=np.linalg.norm(mid[:,i]-mid[:,j],axis=1)
        rate=np.maximum(np.linalg.norm(states[:-1,:,3:5]+drift,axis=2),np.linalg.norm(states[1:,:,3:5]+drift,axis=2))
        radius=(rate[:,i]+rate[:,j])*dt/2+1e-6;low=float(np.maximum(0,distance-radius).min());high=float((distance+radius).max());label=names[i]+'__'+names[j]
        sample=np.r_[np.linalg.norm(states[:,i,:2]-states[:,j,:2],axis=1),distance]
        pairs[label]={'minimum_distance_lower_m':low,'maximum_distance_upper_m':high,'minimum_distance_witness_m':float(sample.min()),'maximum_distance_witness_m':float(sample.max())}
        witness_violation |= sample.min()<25 or(i==0 and sample.max()>2300)
    enclosure=read(args.run_dir/'geometry_enclosures.json');shapes={}
    for name,key,d in [('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir),('navigation','navigation_sha256',args.navigation_dir)]:
        f=d/(name+'.geojson')
        if digest(f)!=cfg[key]:errors.append('geometry source pin')
        shapes[name]=read(f)['features'][0]['geometry']
    lower=[];upper=[];clearances={};navigation={}
    with Geos()as g:
        for j,name in enumerate(names):
            row=enclosure['paths'][name];idx=row['indices'];xy=states[:,j,:2];error=enclosure_error(xy,idx)+float(np.max(np.linalg.norm(commands[:,j,:2],axis=1)*dt**2/8))+1e-6
            if np.max(np.abs(xy[idx]-row['points']))>1e-8 or error>row['error_bound_m']+1e-8:errors.append('unproven path simplification')
            shape={'type':'LineString','coordinates':row['points']};env=g.operation('buffer',shape,distance=error/math.cos(math.pi/480),quadrant_segments=120);navigation[name]=g.covers(shapes['navigation'],env);clearances[name]=g.distance(shape,shapes['hazards'])-error
            witness_violation |= g.distance(shape,shapes['hazards'])+error<50
        if len(enclosure['scans'])!=89:errors.append('full scan geometry accounting')
        for row,w in zip(enclosure['scans'],source['scan_windows']):
            a,b=row['start_index'],row['end_index'];j=names.index(w['platform']);xy=states[a:b+1,j,:2];indices=row['indices']
            if row['task_id']!=w['task_id']or row['platform']!=w['platform']or max(abs(times[a]-w['global_start_s']),abs(times[b]-w['global_end_s']))>1e-7:errors.append('actual scan time mapping')
            error=enclosure_error(xy,indices)+float(np.max(np.linalg.norm(commands[a:b,j,:2],axis=1)*dt[a:b]**2/8))+1e-6
            if np.max(np.abs(xy[indices]-row['points']))>1e-8 or error>row['error_bound_m']+1e-8:errors.append('unproven scan simplification')
            shape={'type':'LineString','coordinates':row['points']};q=cfg['footprint_quadrant_segments']
            lower.append(g.operation('buffer',shape,distance=50-error,quadrant_segments=q));upper.append(g.operation('buffer',shape,distance=(50+error)/math.cos(math.pi/(4*q)),quadrant_segments=q))
        area=g.measure(shapes['searchable'])['area_m2'];cover=lambda shapes_:g.measure(g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':shapes_}),shapes['searchable']))['area_m2']/area
        lo,hi=cover(lower),cover(upper)
    if lo+1e-9<x['geometry']['coverage_lower_ratio']or hi-1e-9>x['geometry']['coverage_upper_ratio']:errors.append('coverage enclosure claim')
    if abs(area-x['geometry']['target_area_m2'])>1e-6:errors.append('original target area')
    certified=all(v['minimum_distance_lower_m']>=25 for v in pairs.values())and max(v['maximum_distance_upper_m']for k,v in pairs.items()if k.startswith('usv__'))<=2300 and all(navigation.values())and min(clearances.values())>=50 and lo>=.999
    witness_violation |= hi<.999
    outcome='certified_pass'if certified else'witnessed_violation'if witness_violation else'unresolved_bounds'
    if x['mission_constraints_passed']and not certified:errors.append('mission pass not independently certified')
    report={'passed':not errors,'errors':errors,'source_result_sha256':digest(args.run_dir/'result.json'),'case':x['case'],'control_intervals':len(dt),'plant_residuals':residual,'maximum_independent_reference_error':max_reference,'maximum_feedback_replay_error_or_short_interval_impulse':controller_error,'distance_pairs':pairs,'independent_coverage_lower_ratio':lo,'independent_coverage_upper_ratio':hi,'independent_hazard_clearance_lower_m':clearances,'navigation_enclosures_passed':navigation,'constraint_outcome':outcome,'mission_constraints_certified':bool(certified),'scope':cfg['scope']}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items()if k!='scope'},indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
