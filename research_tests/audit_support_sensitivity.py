"""A8 independent scheduled phase integration and continuous certificate audit."""
import argparse
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
from audit_curved_cooperation import IndependentClock,audit_partition
from audit_motion_feasibility import piece_points,speed_checks
from audit_headland_navigation import arc_enclosure


class IndependentSchedule:
    def __init__(self,legs,initial):
        self.initial=initial;self.clocks=[IndependentClock(p)for p in legs];self.join_error=0.;prior=np.asarray(initial[:2])
        for c in self.clocks:
            self.join_error+=c.join_error+float(np.linalg.norm(prior-c.state(c.departure)[0]));prior=c.state(c.finish)[0]

    def state(self,t):
        progress=0.;last=(np.asarray(self.initial[:2]),0.,0.,self.initial[2])
        for c in self.clocks:
            if t<c.departure:break
            p,s,v,yaw=c.state(t);last=(p,progress+s,v,yaw)
            if t<c.finish:break
            progress+=c.profile['length_m']
        return last


class IndependentWindow:
    def __init__(self,clock,start):self.clock=clock;self.start=start;self.base=clock.state(start)[1];self.join_error=clock.join_error
    def state(self,t):
        p,s,v,yaw=self.clock.state(t+self.start);return p,s-self.base,v,yaw


def classify_independently(pairs,required,separation,initial):
    if required<initial:return 'initial_condition_impossible'
    for p in pairs:
        if p['minimum_distance_witness_m']<separation or p['maximum_distance_witness_m']>required:return 'witnessed_strategy_violation'
    if pairs and all(p['converged']and p['minimum_distance_lower_m']>=separation and p['maximum_distance_upper_m']<=required for p in pairs):return 'certified_pass'
    return 'unresolved_bounds'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','a7-run-dir','prepared-dir','navigation-dir','json-out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();read=lambda p:json.loads(p.read_text());x=read(args.run_dir/'result.json');source=read(args.a7_run_dir/'result.json');cfg=read(args.run_dir/'config.json');proof=read(args.run_dir/'provenance.json');errors=[]
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    if digest(args.run_dir/'result.json')!=proof['result_sha256']:errors.append('result hash')
    for path,h in proof['input_sha256'].items():
        if digest(Path(path))!=h:errors.append('input hash')
    for path,h in proof['code_sha256'].items():
        if digest(ROOT/path)!=h:errors.append('code hash')
    for path,h in proof['derived_files_sha256'].items():
        if digest(args.run_dir/path)!=h:errors.append('derived hash')
    if digest(args.run_dir/'config.json')not in proof['input_sha256'].values():errors.append('copied config hash')
    canon=lambda value:hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    if canon({k:v for k,v in source.items()if k!='total_computation_seconds'})!=cfg['source_a7_scientific_sha256']:errors.append('A7 scientific signature')
    for name,p in x['uuv_platforms'].items():
        if p!=source['platforms'][name]:errors.append('UUV source geometry/profile/holds changed')
    if x['scan_windows']!=source['scan_windows']or len(x['scan_windows'])!=89 or x['coverage_lower_ratio_unchanged']!=source['coverage_lower_ratio_unchanged']or x['mission_end_s']!=source['mission_end_s']:errors.append('89 windows/coverage/clock inheritance')
    initial=x['mobile_usv']['initial_pose'];legs=x['mobile_usv']['legs'];mobile=IndependentSchedule(legs,initial);uuv={n:IndependentClock(p)for n,p in x['uuv_platforms'].items()};fixed=IndependentClock(source['platforms']['usv']);end=x['mission_end_s']
    if initial!=source['platforms']['usv']['initial_pose']:errors.append('initial USV changed')
    previous=0.;pose=np.asarray(initial);expected_holds=[];length=duration=0.;minimum_clearance=math.inf
    shapes={}
    for name,key,directory in [('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir),('navigation','navigation_sha256',args.navigation_dir)]:
        path=directory/(name+'.geojson')
        if digest(path)!=cfg[key]:errors.append('chart geometry pin')
        shapes[name]=read(path)['features'][0]['geometry']
    with Geos()as g:
        for body,c in zip(legs,mobile.clocks):
            if c.departure<previous-1e-8 or c.finish>end:errors.append('sortie overlap/horizon')
            p=c.state(c.departure)
            if np.linalg.norm(p[0]-pose[:2])>1e-6 or abs(math.remainder(p[3]-pose[2],2*math.pi))>1e-7:errors.append('sortie position/heading discontinuity')
            if c.departure>previous:expected_holds.append((previous,c.departure,list(pose)))
            for a,b in zip(c.pieces,c.pieces[1:]):
                if np.linalg.norm(piece_points(a,[1])[0]-b['pose'][:2])>1e-6 or abs(math.remainder(a['pose'][2]+a['curvature']*a['length']-b['pose'][2],2*math.pi))>1e-7:errors.append('primitive G1 discontinuity')
            if any(p['direction']!=1 or abs(p['curvature'])>1/cfg['usv_radius_m']+1e-12 for p in c.pieces):errors.append('forward R40 primitive')
            errors.extend(speed_checks(c.profile,c.pieces,{'cruise_mps':cfg['usv_cruise_mps'],'tangential_accel_mps2':cfg['tangential_accel_mps2'],'lateral_accel_mps2':cfg['lateral_accel_mps2']}))
            xy,error=arc_enclosure(c.pieces);shape={'type':'LineString','coordinates':xy};envelope=g.operation('buffer',shape,distance=(error+1e-6)/math.cos(math.pi/120),quadrant_segments=30)
            if not g.covers(shapes['navigation'],envelope):errors.append('whole-sortie navigation inclusion')
            clearance=g.distance(shape,shapes['hazards'])-error;minimum_clearance=min(minimum_clearance,clearance)
            if clearance<cfg['hazard_margin_m']or clearance+1e-6<body['minimum_hazard_clearance_lower_m']:errors.append('hazard clearance')
            last=c.state(c.finish);previous=c.finish;pose=np.array([*last[0],last[3]]);length+=c.profile['length_m'];duration+=c.profile['duration_s']
    if previous<end:expected_holds.append((previous,end,list(pose)))
    if len(expected_holds)!=len(x['mobile_usv']['holds']):errors.append('hold count')
    for actual,expected in zip(x['mobile_usv']['holds'],expected_holds):
        if max(abs(actual['t0_s']-expected[0]),abs(actual['t1_s']-expected[1]))>1e-7 or np.linalg.norm(np.asarray(actual['pose'])-expected[2])>1e-6 or actual['scanning']:errors.append('explicit hold record')
    if abs(length-x['mobile_usv']['length_m'])>1e-6 or abs(previous-x['mobile_usv']['motion_finish_s'])>1e-6:errors.append('mobile totals')
    certificates={}
    def check(rel,first,second,horizon,summary=None):
        report=read(args.run_dir/rel);out=audit_partition(report,first,second,horizon);certificates[rel]=out
        if not out['passed']:errors.extend(rel+': '+s for s in out['errors'])
        if summary and any(report[k]!=v for k,v in summary.items()if k!='certificate_file'):errors.append('certificate scalar mismatch')
    inherited_pair=source['distance_pairs']['uuv_1__uuv_2'];inherited_report=read(args.a7_run_dir/inherited_pair['certificate_file'])
    inherited_audit=audit_partition(inherited_report,uuv['uuv_1'],uuv['uuv_2'],end)
    if not inherited_audit['passed']or not inherited_report['converged']or inherited_report['minimum_distance_lower_m']<cfg['minimum_separation_m']:errors.append('inherited UUV pair safety')
    for strategy,clock in [('a7_fixed_station',fixed),('scheduled_mobile',mobile)]:
        for label,pair in x['strategies'][strategy]['pairs'].items():check(pair['certificate_file'],clock,uuv[label.split('__')[1]],end,pair)
    prior=0.;selected_count=0
    for d in x['window_decisions']:
        a,b=d['t0_s'],d['t1_s'];i=d['index'];prefix=IndependentSchedule([p for p in legs if p['departure_s']<a],initial)
        if abs(a-prior)>1e-7 or b<=a:errors.append('window partition gap')
        if d['action']=='transfer':
            selected_count+=1
            if legs[d['leg_index']]['departure_s']!=a:errors.append('selected leg/window departure')
        elif d['action']!='hold':errors.append('unresolved chosen window')
        for n in uuv:
            check(f'certificates/window_{i:02d}_hold_{n}.json',IndependentWindow(prefix,a),IndependentWindow(uuv[n],a),b-a,d['hold_control_pairs']['usv__'+n])
            check(f'certificates/window_{i:02d}_selected_{n}.json',IndependentWindow(mobile,a),IndependentWindow(uuv[n],a),b-a,d['pairs']['usv__'+n])
            p=d['pairs']['usv__'+n]
            if not p['converged']or p['minimum_distance_lower_m']<cfg['minimum_separation_m']or p['maximum_distance_upper_m']>cfg['mobile_design_range_m']:errors.append('selected window constraints')
        prior=b
    if abs(prior-end)>1e-7 or selected_count!=len(legs):errors.append('whole window/leg accounting')
    distances={n:float(np.linalg.norm(mobile.state(0)[0]-c.state(0)[0]))for n,c in uuv.items()};initial_required=max(distances.values());necessary=x['necessary_conditions']
    if any(abs(distances[n]-necessary['initial_usv_uuv_distances_m'][n])>1e-7 for n in uuv)or abs(initial_required-necessary['initial_required_support_range_m'])>1e-7:errors.append('initial necessary lower bound')
    t=necessary['uuv_pair_witness_time_s'];pairbound=np.linalg.norm(uuv['uuv_1'].state(t)[0]-uuv['uuv_2'].state(t)[0])/2
    if abs(pairbound-necessary['uuv_pair_half_maximum_witness_m'])>1e-7:errors.append('pair triangle lower bound')
    for row,required in zip(x['range_sweep'],cfg['range_grid_m']):
        if row['support_range_m']!=required:errors.append('range grid')
        for strategy in x['strategies']:
            if row[strategy]!=classify_independently(list(x['strategies'][strategy]['pairs'].values()),required,cfg['minimum_separation_m'],initial_required):errors.append('range classification')
    if len(x['range_sweep'])!=len(cfg['range_grid_m']):errors.append('range row count')
    with(args.run_dir/'range_sweep.csv').open()as f:
        rows=list(csv.DictReader(f))
        if len(rows)!=len(x['range_sweep'])or any(any(str(v)!=csvrow[k]for k,v in row.items())for row,csvrow in zip(x['range_sweep'],rows)):errors.append('range CSV')
    count=0;max_position=max_speed=max_yaw=0.;trace_times={}
    with(args.run_dir/'mobile_trace.csv').open()as f:
        for row in csv.DictReader(f):
            t=float(row['time_s']);name=row['platform'];clock=mobile if name=='usv'else uuv[name];p,_,v,y=clock.state(t);count+=1;max_position=max(max_position,float(np.linalg.norm(p-[float(row['east_m']),float(row['north_m'])])));max_speed=max(max_speed,abs(v-float(row['speed_mps'])));max_yaw=max(max_yaw,abs(math.remainder(y-float(row['heading_rad']),2*math.pi)));trace_times.setdefault(t,set()).add(name)
            task=next((w['task_id']for w in source['scan_windows']if w['platform']==name and w['global_start_s']<=t<w['global_end_s']),'')
            if row['active_task_id']!=task:errors.append('trace scan state')
            if name=='usv':
                started=[c for c in mobile.clocks if c.departure<=t]
                mode='predeparture_hold'if not started else'moving'if t<started[-1].finish else'completed_hold'if len(started)==len(mobile.clocks)else'interleg_hold'
            else:mode='predeparture_hold'if t<clock.departure else'completed_hold'if t>=clock.finish else'moving'
            if row['mode']!=mode:errors.append('trace hold/moving mode')
    if max_position>1e-6 or max_speed>1e-7 or max_yaw>1e-7:errors.append('independent trace states')
    if count!=3*len(trace_times)or any(s!={'usv','uuv_1','uuv_2'}for s in trace_times.values())or min(trace_times)!=0 or abs(max(trace_times)-end)>1e-7:errors.append('trace full common horizon')
    report={'passed':not errors,'errors':errors,'source_result_sha256':digest(args.run_dir/'result.json'),'mission_end_s':end,'all_source_initial_poses_uuv_profiles_and89_windows_preserved':True,'independent_certificate_count':len(certificates),'certificate_audits':certificates,'initial_required_range_m':initial_required,'pair_half_distance_witness_m':float(pairbound),'mobile_sorties':len(legs),'mobile_length_m':length,'mobile_motion_duration_s':duration,'mobile_hazard_clearance_lower_m':minimum_clearance,'trace_rows_checked':count,'maximum_position_error_m':max_position,'maximum_speed_error_mps':max_speed,'maximum_heading_error_rad':max_yaw,'range_grid_independently_classified':True,'scope':'Deterministic offline2D scheduled USV comparison; inherited UUV curves/coverage. Ideal holds, geometric range. No current, energy, real acoustics/depth/controller/deployment/recovery verification.'}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items()if k!='certificate_audits'},indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
