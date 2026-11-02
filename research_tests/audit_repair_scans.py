"""Independent A6 repair-task accounting, clocks, greedy gains and sensor bounds."""
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
from audit_headland_navigation import arc_enclosure
from audit_joint_coverage import distance_to_piece,straight_capsule_upper
from audit_open_chart_stage_a import inside
from audit_motion_feasibility import piece_points


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('repair-dir','run-dir','route-audit-json','prepared-dir','json-out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();x=json.loads((args.run_dir/'result.json').read_text());repair=json.loads((args.repair_dir/'result.json').read_text())
    cfg=json.loads((args.repair_dir/'config.json').read_text());proof=json.loads((args.repair_dir/'provenance.json').read_text());route_audit=json.loads(args.route_audit_json.read_text());errors=[]
    digest=hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()
    science={k:v for k,v in x.items()if k!='total_computation_seconds'}
    if hashlib.sha256(json.dumps(science,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()!=cfg['source_a6_scientific_sha256']:errors.append('A6 scientific signature')
    if not route_audit['passed']or route_audit['source_result_sha256']!=digest or repair['source_result_sha256']!=digest:errors.append('route audit mismatch')
    if hashlib.sha256((args.repair_dir/'result.json').read_bytes()).hexdigest()!=proof['result_sha256']:errors.append('repair result hash')
    if hashlib.sha256((args.repair_dir/'config.json').read_bytes()).hexdigest()not in proof['input_sha256'].values():errors.append('copied repair config')
    for p,h in proof['input_sha256'].items():
        if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h:errors.append('input hash')
    for p,h in proof['code_sha256'].items():
        if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h:errors.append('code hash')
    for p,h in proof['derived_files_sha256'].items():
        if hashlib.sha256((args.repair_dir/p).read_bytes()).hexdigest()!=h:errors.append('derived hash')
    targetpath=args.prepared_dir/'searchable.geojson'
    if hashlib.sha256(targetpath.read_bytes()).hexdigest()!=cfg['target_geometry_sha256']:errors.append('target hash')
    target=json.loads(targetpath.read_text())['features'][0]['geometry'];selected=repair['selected_repairs'];ids=[r['task_id']for r in selected]
    if len(set(ids))!=len(selected)or len(selected)!=repair['explicit_repair_task_count']:errors.append('repair task identifiers/count')
    scanpieces=[];chosenpieces=[];candidate_pieces={};time_windows={};source_ids=[]
    for name in ('uuv_1','uuv_2'):
        plan=x['platforms'][name];offset=0
        for index,leg in enumerate(plan['legs']):
            count=len(leg['pieces'])
            if leg['scanning']:source_ids.append(leg['task_id']);scanpieces.extend(leg['pieces'])
            elif count:
                key=(name,index);candidate_pieces[key]=leg['pieces'];segments=plan['profile']['segments']
                time_windows[key]=(offset,offset+count-1,segments[offset]['t0_s'],segments[offset+count-1]['t1_s'])
            offset+=count
    if len(source_ids)!=86 or len(set(source_ids))!=86 or repair['original_scan_task_count']!=86 or repair['total_declared_scan_tasks']!=86+len(selected):errors.append('original task accounting')
    chosen_keys=[(r['platform'],r['source_transit_leg_index'])for r in selected]
    if len(set(chosen_keys))!=len(chosen_keys):errors.append('duplicate repair interval')
    duration=0.
    for r,key in zip(selected,chosen_keys):
        if key not in candidate_pieces:errors.append('repair not on original transit');continue
        first,last,t0,t1=time_windows[key]
        if r['first_piece_index']!=first or r['last_piece_index']!=last or max(abs(r['local_start_s']-t0),abs(r['local_end_s']-t1),abs(r['active_scan_duration_s']-(t1-t0)))>1e-7:errors.append('repair scan window')
        duration+=t1-t0;chosenpieces.extend(candidate_pieces[key])
    if abs(duration-repair['additional_active_scanning_duration_s'])>1e-7:errors.append('active scanning total')
    if repair['added_motion_distance_m']!=0 or repair['added_motion_time_s']!=0 or not repair['motion_profiles_unchanged']:errors.append('unexpected route alteration claim')
    if not repair['other_transits_scanning_OFF']or repair['cooperative_mission_clock_released']:errors.append('unexpected policy/clock claim')
    with Geos()as g:
        area=g.measure(target)['area_m2'];base_shapes=[];base_upper=[]
        for p in scanpieces:
            endpoints=[p['pose'][:2],piece_points(p,[1])[0].tolist()];base_shapes.append(g.operation('buffer',{'type':'LineString','coordinates':endpoints},distance=50,quadrant_segments=30));base_upper.append(straight_capsule_upper(g,endpoints,50))
        base=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':base_shapes}),target)
        coarse={};fine_lo=[];fine_hi=[]
        for key,pp in candidate_pieces.items():
            xy,bound=arc_enclosure(pp,error=.02);coarse[key]=g.operation('intersection',g.operation('buffer',{'type':'LineString','coordinates':xy},distance=50-bound,quadrant_segments=30),target)
            if key in chosen_keys:
                xy,bound=arc_enclosure(pp,error=.01);shape={'type':'LineString','coordinates':xy}
                fine_lo.append(g.operation('buffer',shape,distance=50-bound,quadrant_segments=30));fine_hi.append(g.operation('buffer',shape,distance=(50+bound)/math.cos(math.pi/120),quadrant_segments=30))
        current=base;remaining=set(coarse);marginal_records=[]
        for record,key in zip(selected,chosen_keys):
            gains={k:g.measure(g.operation('difference',coarse[k],current))['area_m2']for k in remaining};gain=gains[key]
            if gain+0.01<max(gains.values())or abs(gain-record['marginal_covered_area_m2'])>.01:errors.append('greedy marginal gain')
            current=g.operation('union',{'type':'GeometryCollection','geometries':[current,coarse[key]]});remaining.remove(key)
            ratio=g.measure(current)['area_m2']/area
            if abs(ratio-record['cumulative_coverage_lower_ratio'])>1e-9:errors.append('cumulative footprint')
            marginal_records.append({'task_id':record['task_id'],'independent_marginal_area_m2':gain,'cumulative_coverage_lower_ratio':ratio})
        lo=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':[base]+fine_lo}),target)
        hi=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':base_upper+fine_hi}),target)
        lo_ratio=g.measure(lo)['area_m2']/area;hi_ratio=g.measure(hi)['area_m2']/area
        if abs(g.measure(current)['area_m2']/area-repair['coverage_lower_ratio'])>1e-9:errors.append('reported lower coverage')
        if lo_ratio>repair['coverage_upper_ratio']+1e-9 or hi_ratio<repair['coverage_lower_ratio']-1e-9:errors.append('independent intervals disjoint')
    axis=np.arange(2.5,5120,5);xx,yy=np.meshgrid(axis,axis);points=np.column_stack((xx.ravel(),yy.ravel()));mask,boundary=inside(target,points);points=points[mask&~boundary];distance=np.full(len(points),np.inf)
    for p in scanpieces+chosenpieces:
        if p['curvature']:
            k=p['curvature'];yaw=p['pose'][2];centre=np.asarray(p['pose'][:2])+[-math.sin(yaw)/k,math.cos(yaw)/k];low=centre-1/abs(k);high=centre+1/abs(k)
        else:
            ends=np.array([p['pose'][:2],piece_points(p,[1])[0]]);low=ends.min(axis=0);high=ends.max(axis=0)
        subset=np.flatnonzero(np.all((points>=low-50.1)&(points<=high+50.1),axis=1));distance[subset]=np.minimum(distance[subset],distance_to_piece(points[subset],p))
    saved_lower,_=inside(json.loads((args.repair_dir/'repair_covered_lower.geojson').read_text()),points);saved_upper,_=inside(json.loads((args.repair_dir/'repair_covered_upper.geojson').read_text()),points)
    if np.any(saved_lower&(distance>50+1e-6))or np.any((distance<50-1e-6)&~saved_upper):errors.append('exact sensor sample outside footprint bounds')
    gate=min(lo_ratio,repair['coverage_lower_ratio'])>=cfg['required_coverage_ratio']
    if not gate or not repair['coverage_gate_passed']:errors.append('repair coverage below declared gate')
    report={'passed':not errors,'errors':errors,'route_source_result_sha256':digest,'repair_source_result_sha256':proof['result_sha256'],
            'original_tasks_verified':86,'explicit_repair_tasks_verified':len(selected),'total_scan_tasks_verified':86+len(selected),'marginal_records':marginal_records,
            'coverage_lower_ratio':lo_ratio,'coverage_upper_ratio':hi_ratio,'coverage_gate_passed':gate,'target_area_m2':area,
            'additional_active_scanning_duration_s':duration,'independent_sample_spacing_m':5,'independent_target_sample_count':len(points),'exact_sensor_sample_coverage_ratio':float(np.mean(distance<=50)),
            'route_and_motion_profiles_unchanged':True,'cooperative_clock_and_separation_verified':False,
            'scope':'86 original scans plus listed curved repair activations, other transit OFF; ideal horizontal50m sensing, full unchanged target; not real sonar or complete cooperative vehicle feasibility.'}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
