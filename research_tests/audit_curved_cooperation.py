"""A7 independent clock integration and whole-time interval certificate audit.

No production clock, Piece evaluation or distance-certification imports.
Circle centres and phase-integral sums provide a separate implementation.
"""
import argparse
from bisect import bisect_right
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from chart_geometry import Geos
from audit_motion_feasibility import piece_points,speed_checks
from audit_headland_navigation import arc_enclosure
from audit_joint_coverage import distance_to_piece


class IndependentClock:
    def __init__(self,platform):
        self.pieces=platform['pieces'];self.profile=platform['profile'];self.departure=platform.get('departure_s',0.);self.finish=self.departure+self.profile['duration_s']
        self.starts=[s['t0_s']for s in self.profile['segments']];self.initial=platform.get('initial_pose',self.pieces[0]['pose']if self.pieces else None)
        self.join_error=sum(float(np.linalg.norm(piece_points(a,[1])[0]-b['pose'][:2]))for a,b in zip(self.pieces,self.pieces[1:]))

    def state(self,time):
        local=time-self.departure
        if not self.pieces:return np.asarray(self.initial[:2]),0.,0.,self.initial[2]
        if local<=0:return np.asarray(self.pieces[0]['pose'][:2]),0.,0.,self.pieces[0]['pose'][2]
        if local>=self.profile['duration_s']:
            p=self.pieces[-1];return piece_points(p,[1])[0],self.profile['length_m'],0.,p['pose'][2]+p['curvature']*p['length']
        i=max(0,bisect_right(self.starts,local)-1);s=self.profile['segments'][i];p=self.pieces[i];t=local-s['t0_s'];ta,tc,a,v=s['ramp_duration_s'],s['cruise_duration_s'],s['accel_mps2'],s['peak_speed_mps']
        accelerating=min(t,ta);cruising=min(max(t-ta,0.),tc);decelerating=min(max(t-ta-tc,0.),ta)
        travel=.5*a*accelerating**2+v*cruising+v*decelerating-.5*a*decelerating**2;speed=a*accelerating-a*decelerating
        return piece_points(p,[travel/p['length']])[0],s['s0_m']+travel,speed,p['pose'][2]+p['curvature']*travel


def audit_partition(report,first,second,end):
    errors=[];prior=0.;low=math.inf;high=-math.inf;maximum_radius_error=0.;margin=report['numerical_margin_m']
    for row in report['partition']:
        a,b,m=row['t0_s'],row['t1_s'],row['midpoint_s']
        if abs(a-prior)>1e-7 or b<=a or abs(m-(a+b)/2)>1e-8:errors.append('partition gap/order')
        left=[clock.state(a)for clock in(first,second)];middle=[clock.state(m)for clock in(first,second)];right=[clock.state(b)for clock in(first,second)]
        distance=float(np.linalg.norm(middle[0][0]-middle[1][0]));radius=max(sum(mid[1]-lo[1]for mid,lo in zip(middle,left)),sum(hi[1]-mid[1]for hi,mid in zip(right,middle)),0.)+margin+first.join_error+second.join_error
        maximum_radius_error=max(maximum_radius_error,abs(radius-row['relative_movement_bound_m']))
        if abs(distance-row['midpoint_distance_m'])>1e-6 or abs(radius-row['relative_movement_bound_m'])>1e-6:errors.append('independent midpoint/radius mismatch')
        if row['minimum_distance_lower_m']>max(0.,distance-radius)+1e-7 or row['maximum_distance_upper_m']<distance+radius-1e-7:errors.append('unsafe interval bound')
        low=min(low,row['minimum_distance_lower_m']);high=max(high,row['maximum_distance_upper_m']);prior=b
    if abs(prior-end)>1e-7 or not report['partition']:errors.append('partition horizon')
    for key,timekey in [('minimum_distance_witness_m','minimum_witness_time_s'),('maximum_distance_witness_m','maximum_witness_time_s')]:
        t=report[timekey]
        if not 0<=t<=end:errors.append('witness outside horizon');continue
        actual=float(np.linalg.norm(first.state(t)[0]-second.state(t)[0]))
        if abs(actual-report[key])>1e-6:errors.append('witness mismatch')
    if abs(low-report['minimum_distance_lower_m'])>1e-7 or abs(high-report['maximum_distance_upper_m'])>1e-7:errors.append('global interval extrema')
    if report['converged']and max(report['minimum_distance_witness_m']-low,high-report['maximum_distance_witness_m'])>report['tolerance_m']+1e-6:errors.append('unmet convergence tolerance')
    return {'passed':not errors,'errors':errors,'interval_count':len(report['partition']),'maximum_independent_radius_error_m':maximum_radius_error}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','a6-run-dir','repair-dir','prepared-dir','navigation-dir','json-out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();x=json.loads((args.run_dir/'result.json').read_text());cfg=json.loads((args.run_dir/'config.json').read_text());proof=json.loads((args.run_dir/'provenance.json').read_text());source=json.loads((args.a6_run_dir/'result.json').read_text());repair=json.loads((args.repair_dir/'result.json').read_text());errors=[]
    digest=hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()
    if digest!=proof['result_sha256']:errors.append('result hash')
    if hashlib.sha256((args.run_dir/'config.json').read_bytes()).hexdigest()not in proof['input_sha256'].values():errors.append('copied config hash')
    for path,h in proof['input_sha256'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=h:errors.append('input hash')
    for path,h in proof['code_sha256'].items():
        if hashlib.sha256((ROOT/path).read_bytes()).hexdigest()!=h:errors.append('code hash')
    for path,h in proof['derived_files_sha256'].items():
        if hashlib.sha256((args.run_dir/path).read_bytes()).hexdigest()!=h:errors.append('derived hash')
    canon=lambda value:hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    if canon({k:v for k,v in source.items()if k!='total_computation_seconds'})!=cfg['source_a6_scientific_sha256']or canon({k:v for k,v in repair.items()if k!='source_result_sha256'})!=cfg['source_repair_scientific_sha256']:errors.append('scientific source signature')
    for name in('uuv_1','uuv_2'):
        if x['platforms'][name]['pieces']!=source['platforms'][name]['pieces']or x['platforms'][name]['profile']!=source['platforms'][name]['profile']:errors.append('changed UUV geometry/profile')
    clocks={name:IndependentClock(p)for name,p in x['platforms'].items()};end=x['mission_end_s']
    if abs(end-max(c.finish for c in clocks.values()))>1e-7:errors.append('mission end')
    for name,p in x['platforms'].items():
        if np.linalg.norm(clocks[name].state(0)[0]-source['platforms'][name]['pieces'][0]['pose'][:2])>1e-7 or abs(math.remainder(clocks[name].state(0)[3]-source['platforms'][name]['pieces'][0]['pose'][2],2*math.pi))>1e-8:errors.append('changed initial pose')
        expected_holds=[]
        if p['departure_s']>0:expected_holds.append((0.,p['departure_s'],'predeparture_hold'))
        if clocks[name].finish<end:expected_holds.append((clocks[name].finish,end,'completed_hold'))
        if [(h['t0_s'],h['t1_s'],h['kind'])for h in p['holds']]!=expected_holds or any(h['scanning']for h in p['holds']):errors.append('hold schedule')
        for hold in p['holds']:
            if np.linalg.norm(clocks[name].state((hold['t0_s']+hold['t1_s'])/2)[0]-hold['pose'][:2])>1e-6:errors.append('hold position')
    expected={}
    for name in('uuv_1','uuv_2'):
        p=source['platforms'][name];offset=0
        for index,leg in enumerate(p['legs']):
            count=len(leg['pieces'])
            if leg['scanning']:expected[(name,leg['task_id'])]=(index,p['profile']['segments'][offset]['t0_s'],p['profile']['segments'][offset+count-1]['t1_s'],'source_scan')
            offset+=count
    for r in repair['selected_repairs']:expected[(r['platform'],r['task_id'])]=(r['source_transit_leg_index'],r['local_start_s'],r['local_end_s'],'repair_scan')
    if len(x['scan_windows'])!=89 or {(w['platform'],w['task_id'])for w in x['scan_windows']}!=set(expected):errors.append('89 scan windows accounting')
    for w in x['scan_windows']:
        index,a,b,kind=expected[(w['platform'],w['task_id'])];c=clocks[w['platform']]
        if index!=w['source_leg_index']or kind!=w['kind']or max(abs(w['local_start_s']-a),abs(w['local_end_s']-b),abs(w['global_start_s']-a-c.departure),abs(w['global_end_s']-b-c.departure))>1e-7 or not(c.departure<=w['global_start_s']<w['global_end_s']<=c.finish+1e-7):errors.append('global scan mapping')
    with(args.run_dir/'global_scan_schedule.csv').open()as f:csv_scans=list(csv.DictReader(f))
    if len(csv_scans)!=89 or {(r['platform'],r['task_id'])for r in csv_scans}!=set(expected):errors.append('scan CSV accounting')
    for row,w in zip(csv_scans,x['scan_windows']):
        if row['task_id']!=w['task_id']or any(abs(float(row[k])-w[k])>1e-7 for k in ('local_start_s','local_end_s','global_start_s','global_end_s')):errors.append('scan CSV window')
    naive=json.loads((args.run_dir/'naive_baseline.json').read_text());oldclocks={name:IndependentClock(p)for name,p in source['platforms'].items()};oldend=max(c.finish for c in oldclocks.values());certificate_audits={};baseline_witness_violations=[]
    def check_pair(summary,a,b,horizon,label):
        report=json.loads((args.run_dir/summary['certificate_file']).read_text());audit=audit_partition(report,a,b,horizon);certificate_audits[label]=audit
        if not audit['passed']:errors.extend(label+': '+e for e in audit['errors'])
        if any(report[k]!=v for k,v in summary.items()if k!='certificate_file'):errors.append('certificate summary mismatch')
    for label,pair in naive['pairs'].items():
        a,b=label.split('__');check_pair(pair,oldclocks[a],oldclocks[b],oldend,'naive_'+label)
        if pair['minimum_distance_witness_m']<cfg['minimum_separation_m']or('usv'in label and pair['maximum_distance_witness_m']>cfg['support_range_m']):baseline_witness_violations.append(label)
    if naive['all_constraints_certified']or not baseline_witness_violations:errors.append('naive negative control not witnessed')
    for i,trial in enumerate(x['uuv_departure_trials']):
        second=IndependentClock({**source['platforms']['uuv_2'],'departure_s':trial['uuv2_departure_s']});horizon=max(oldclocks['uuv_1'].finish,second.finish)
        check_pair(trial['pair'],oldclocks['uuv_1'],second,horizon,f'departure_{i}')
    for i,attempt in enumerate(x['support_attempts']):
        surface=IndependentClock(attempt['usv_motion']);trialclocks={**clocks,'usv':surface};horizon=max(c.finish for c in trialclocks.values())
        for label,pair in attempt['pairs'].items():
            a,b=label.split('__');check_pair(pair,trialclocks[a],trialclocks[b],horizon,f'support_{i}_{label}')
    for label,pair in x['distance_pairs'].items():
        a,b=label.split('__');check_pair(pair,clocks[a],clocks[b],end,'chosen_'+label)
        if not pair['converged']or pair['minimum_distance_lower_m']<25 or('usv'in label and pair['maximum_distance_upper_m']>3000):errors.append('chosen common clock violates constraints')
    for path,key in [(args.navigation_dir/'navigation.geojson','navigation_sha256'),(args.prepared_dir/'hazards.geojson','hazards_geometry_sha256'),(args.prepared_dir/'searchable.geojson','target_geometry_sha256')]:
        if hashlib.sha256(path.read_bytes()).hexdigest()!=cfg[key]:errors.append('audit chart pin')
    nav=json.loads((args.navigation_dir/'navigation.geojson').read_text())['features'][0]['geometry'];hazards=json.loads((args.prepared_dir/'hazards.geojson').read_text())['features'][0]['geometry'];minimum_clearance=math.inf
    with Geos()as g:
        for name,p in x['platforms'].items():
            limits={'cruise_mps':cfg['usv_cruise_mps']if name=='usv'else source['platforms'][name]['profile']['cruise_mps'],'tangential_accel_mps2':cfg['tangential_accel_mps2'],'lateral_accel_mps2':cfg['lateral_accel_mps2']}
            errors.extend(name+': '+e for e in speed_checks(p['profile'],p['pieces'],limits));radius=40 if name=='usv'else 20
            for prior,cur in zip(p['pieces'],p['pieces'][1:]):
                if np.linalg.norm(piece_points(prior,[1])[0]-cur['pose'][:2])>1e-6 or abs(math.remainder(prior['pose'][2]+prior['curvature']*prior['length']-cur['pose'][2],2*math.pi))>1e-7:errors.append('route G1 continuity')
            if any(v['direction']!=1 or abs(v['curvature'])>1/radius+1e-12 for v in p['pieces']):errors.append('forward turning radius')
            xy,bound=arc_enclosure(p['pieces']);chord={'type':'LineString','coordinates':xy};envelope=g.operation('buffer',chord,distance=(bound+1e-6)/math.cos(math.pi/120),quadrant_segments=30)
            if not g.covers(nav,envelope):errors.append('independent whole-curve navigation inclusion')
            minimum_clearance=min(minimum_clearance,g.distance(chord,hazards)-bound)
        if minimum_clearance<50:errors.append('50m hazard margin')
        station=np.asarray(x['support_station']['position_m']);station_min=math.inf;station_max=0.
        if not g.covers(nav,{'type':'Point','coordinates':station.tolist()})or np.linalg.norm(clocks['usv'].state(end)[0]-station)>1e-6:errors.append('support station position')
        for name in('uuv_1','uuv_2'):
            for p in x['platforms'][name]['pieces']:
                station_min=min(station_min,float(distance_to_piece(station[None,:],p)[0]));a=np.asarray(p['pose'][:2]);b=piece_points(p,[1])[0];farthest=max(np.linalg.norm(a-station),np.linalg.norm(b-station))
                if p['curvature']:
                    k=p['curvature'];yaw=p['pose'][2];centre=a+[-math.sin(yaw)/k,math.cos(yaw)/k];direction=math.atan2(centre[1]-station[1],centre[0]-station[0]);startangle=math.atan2(a[1]-centre[1],a[0]-centre[0])
                    if (math.copysign(1,k)*(direction-startangle))%(2*math.pi)<=abs(k)*p['length']+1e-12:farthest=max(farthest,np.linalg.norm(centre-station)+1/abs(k))
                station_max=max(station_max,float(farthest))
        if station_min+1e-6<x['support_station']['entire_uuv_route_clearance_lower_m']or station_max>x['support_station']['worst_entire_route_range_upper_m']+1e-6:errors.append('station whole-route geometric bounds')
    max_position_error=max_speed_error=max_heading_error=0.;trace_times={};trace_rows=0
    with(args.run_dir/'common_clock_trace.csv').open()as f:
        for row in csv.DictReader(f):
            trace_rows+=1;t=float(row['time_s']);name=row['platform'];position,_,speed,yaw=clocks[name].state(t);error=float(np.linalg.norm(position-[float(row['east_m']),float(row['north_m'])]));max_position_error=max(max_position_error,error);max_speed_error=max(max_speed_error,abs(speed-float(row['speed_mps'])));max_heading_error=max(max_heading_error,abs(math.remainder(yaw-float(row['heading_rad']),2*math.pi)))
            active=next((w['task_id']for w in x['scan_windows']if w['platform']==name and w['global_start_s']<=t<w['global_end_s']),'')
            if row['active_task_id']!=active or row['scanning']!=str(bool(active)):errors.append('trace scanning status')
            mode='predeparture_hold'if t<clocks[name].departure else'completed_hold'if t>=clocks[name].finish else'moving'
            if row['mode']!=mode:errors.append('trace hold/move mode')
            trace_times.setdefault(t,set()).add(name)
    if any(v!={'uuv_1','uuv_2','usv'}for v in trace_times.values())or trace_rows!=3*len(trace_times)or min(trace_times)!=0 or abs(max(trace_times)-end)>1e-7:errors.append('trace common clock coverage')
    if max_position_error>1e-6 or max_speed_error>1e-7 or max_heading_error>1e-7:errors.append('independent trace state mismatch')
    if x['coverage_lower_ratio_unchanged']!=repair['coverage_lower_ratio']:errors.append('coverage inheritance')
    report={'passed':not errors,'errors':errors,'source_result_sha256':digest,'mission_end_s':end,'all_initial_poses_preserved':True,'source86_and_repair3_windows_verified':True,
            'certificate_audits':certificate_audits,'naive_baseline_witnessed_violations':baseline_witness_violations,'minimum_hazard_clearance_lower_m':minimum_clearance,
            'trace_rows_independently_checked':trace_rows,'maximum_trace_position_error_m':max_position_error,'maximum_trace_speed_error_mps':max_speed_error,'maximum_trace_heading_error_rad':max_heading_error,
            'independent_station_minimum_whole_route_distance_m':station_min,'independent_station_maximum_whole_route_distance_m':station_max,
            'continuous_model_constraints_certified':not errors,'coverage_lower_ratio_inherited':repair['coverage_lower_ratio'],
            'scope':'All-time2D point kinematics with ideal station holding and geometric3000m range. No current/depth/acoustic/hull/controller or deployment/recovery verification.'}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
