"""Add explicit ideal curved repair scans on already audited A6 transit legs.

All original86 scans remain. Trajectories/profiles are immutable; unselected
transits remain scanning OFF. Greedy area gain is not a minimum-set proof.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece
from chart_geometry import Geos
from run_motion_feasibility import sample_enclosure
from scan_footprints import straight_footprint_bounds


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','audit-json','prepared-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/coverage_repair_a6.json')
    args=parser.parse_args();cfg=json.loads(args.config.read_text());sourcepath=args.run_dir/'result.json';x=json.loads(sourcepath.read_text());audit=json.loads(args.audit_json.read_text())
    digest=hashlib.sha256(sourcepath.read_bytes()).hexdigest();science={k:v for k,v in x.items()if k!='total_computation_seconds'}
    if hashlib.sha256(json.dumps(science,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()!=cfg['source_a6_scientific_sha256']:raise ValueError('A6 scientific route changed')
    if not audit['passed']or audit['source_result_sha256']!=digest or not x['complete_platform_routes_found']:raise ValueError('Complete matching audited source required')
    targetpath=args.prepared_dir/'searchable.geojson'
    if hashlib.sha256(targetpath.read_bytes()).hexdigest()!=cfg['target_geometry_sha256']:raise ValueError('Original target changed')
    target=json.loads(targetpath.read_text())['features'][0]['geometry'];args.output_dir.mkdir(parents=True,exist_ok=False)
    (args.output_dir/'config.json').write_bytes(args.config.read_bytes());candidates=[];source_scan_count=0
    with Geos()as g:
        area=g.measure(target)['area_m2'];scans=[];upper_scans=[]
        for name in ('uuv_1','uuv_2'):
            platform=x['platforms'][name];offset=0
            for index,leg in enumerate(platform['legs']):
                pieces=leg['pieces'];count=len(pieces)
                if leg['scanning']:
                    source_scan_count+=1;lo,hi=straight_footprint_bounds(g,leg['points'],cfg['sensor_radius_m'],cfg['footprint_quadrant_segments']);scans.append(lo);upper_scans.append(hi)
                elif count:
                    xy,error=sample_enclosure([Piece(**p)for p in pieces],cfg['maximum_arc_sagitta_m']);shape={'type':'LineString','coordinates':xy}
                    lower=g.operation('intersection',g.operation('buffer',shape,distance=cfg['sensor_radius_m']-error,quadrant_segments=30),target)
                    upper=g.operation('intersection',g.operation('buffer',shape,distance=(cfg['sensor_radius_m']+error)/math.cos(math.pi/120),quadrant_segments=30),target)
                    segments=platform['profile']['segments'];t0=segments[offset]['t0_s'];t1=segments[offset+count-1]['t1_s']
                    candidates.append({'platform':name,'source_transit_leg_index':index,'first_piece_index':offset,'last_piece_index':offset+count-1,
                                       'local_start_s':t0,'local_end_s':t1,'active_scan_duration_s':t1-t0,'lower':lower,'upper':upper})
                offset+=count
        covered=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':scans}),target);initial=covered
        original_area=g.measure(covered)['area_m2']
        if abs(original_area/area-x['coverage']['scan_only_coverage_ratio'])>1e-10:raise ValueError('Original scan footprint mismatch')
        selected=[];steps=[];covered_area=original_area;remaining=list(range(len(candidates)))
        while covered_area/area<cfg['required_coverage_ratio']:
            gains=[(g.measure(g.operation('difference',candidates[i]['lower'],covered))['area_m2'],-candidates[i]['active_scan_duration_s'],-i,i)for i in remaining]
            if not gains:break
            gain,_,_,index=max(gains)
            if gain<=1e-6:break
            v=candidates[index];covered=g.operation('union',{'type':'GeometryCollection','geometries':[covered,v['lower']]});covered_area=g.measure(covered)['area_m2'];remaining.remove(index)
            selected.append(index);record={k:value for k,value in v.items()if k not in ('lower','upper')};record.update({'task_id':f'repair_{len(selected):03d}','marginal_covered_area_m2':gain,'cumulative_coverage_lower_ratio':covered_area/area});steps.append(record)
        upper=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':upper_scans+[candidates[i]['upper']for i in selected]}),target)
        result={'stage':'A6 explicit repair scanning','coverage_gate_passed':covered_area/area>=cfg['required_coverage_ratio'],
                'source_result_sha256':digest,'original_scan_task_count':source_scan_count,'explicit_repair_task_count':len(selected),'total_declared_scan_tasks':source_scan_count+len(selected),
                'selected_repairs':steps,'target_area_m2':area,'original_scan_only_coverage_ratio':original_area/area,'coverage_lower_ratio':covered_area/area,
                'coverage_upper_ratio':g.measure(upper)['area_m2']/area,'missed_area_upper_bound_m2':area-covered_area,'required_coverage_ratio':cfg['required_coverage_ratio'],
                'additional_active_scanning_duration_s':sum(t['active_scan_duration_s']for t in steps),'added_motion_distance_m':0.,'added_motion_time_s':0.,
                'motion_profiles_unchanged':True,'other_transits_scanning_OFF':True,'cooperative_mission_clock_released':False,
                'policy':'Original86 straight scans plus listed curved repair scans on existing transits. Ideal horizontal50m circle. No change to target, route geometry or independent motion profiles. Greedy largest guaranteed marginal area; not a minimum repair-set proof.'}
        for filename,shape in [('original_scan_covered.geojson',initial),('repair_covered_lower.geojson',covered),('repair_covered_upper.geojson',upper),('repair_missed_upper.geojson',g.operation('difference',target,covered))]:
            (args.output_dir/filename).write_text(json.dumps(shape)+'\n')
    (args.output_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    with(args.output_dir/'repair_schedule.csv').open('w',newline='')as f:
        fields=['task_id','platform','source_transit_leg_index','first_piece_index','last_piece_index','local_start_s','local_end_s','active_scan_duration_s'];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        writer.writerows({key:step[key]for key in fields}for step in sorted(steps,key=lambda s:(s['platform'],s['local_start_s'])))
    files=[args.config,sourcepath,args.audit_json,targetpath];code=[Path(__file__),ROOT/'scripts/chart_geometry.py',ROOT/'scripts/run_motion_feasibility.py',ROOT/'scripts/scan_footprints.py',ROOT/'src/berthing_planner/curves.py']
    provenance={'input_sha256':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in files},'code_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in code},
                'result_sha256':hashlib.sha256((args.output_dir/'result.json').read_bytes()).hexdigest(),'derived_files_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in args.output_dir.iterdir()if p.name not in ('result.json','config.json')}}
    (args.output_dir/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
