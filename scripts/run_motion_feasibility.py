"""A4: certify fixed-scan turning obstructions, then test a local headland arc."""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece, line
from berthing_planner.motion_feasibility import convex_hull, forward_turn_witnesses, stopping_profile, profile_state
from chart_geometry import Geos


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


from berthing_planner.experiment.curve_geometry import sample_enclosure


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a3-run-dir',type=Path,required=True)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/motion_feasibility_a4.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    cfg=json.loads(args.config.read_text())
    positive=('nominal_uuv_minimum_radius_m','nominal_usv_minimum_radius_m','local_headland_radius_m',
              'local_scan_spacing_m','local_segment_length_m','maximum_arc_sagitta_m',
              'cruise_mps','tangential_accel_mps2','lateral_accel_mps2','sensor_radius_m')
    if (any(not math.isfinite(cfg[k])or cfg[k]<=0 for k in positive)
            or not cfg['minimum_radius_sweep_m']
            or any(not math.isfinite(v)or v<=0 for v in cfg['minimum_radius_sweep_m'])
            or cfg['nominal_uuv_minimum_radius_m']not in cfg['minimum_radius_sweep_m']
            or cfg['local_headland_radius_m']<cfg['nominal_uuv_minimum_radius_m']
            or cfg['local_scan_spacing_m']<2*cfg['local_headland_radius_m']
            or not 0<=cfg['support_hull_relative_expansion']
            or not 0<=cfg['local_headland_extra_inset_m']
            or not cfg['maximum_arc_sagitta_m']<min(cfg['local_headland_radius_m'],cfg['sensor_radius_m'])):
        raise ValueError('Invalid finite-radius workspace/speed configuration')
    for path,expected in [(args.prepared_dir/'searchable.geojson',cfg['target_candidate_sha256']),
                          (args.prepared_dir/'hazards.geojson',cfg['a1_hazards_sha256']),
                          (args.prepared_dir/'task_boundary.geojson',cfg['a1_task_boundary_sha256'])]:
        if sha(path)!=expected:raise ValueError('Fixed A3/A1 input changed: '+path.name)
    source=json.loads((args.a3_run_dir/'result.json').read_text())
    scientific={k:v for k,v in source.items()if k!='total_computation_seconds'}
    scientific_hash=hashlib.sha256(json.dumps(scientific,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    source_provenance=json.loads((args.a3_run_dir/'provenance.json').read_text())
    if scientific_hash!=cfg['source_a3_scientific_result_sha256']or source_provenance['result_sha256']!=sha(args.a3_run_dir/'result.json'):
        raise ValueError('A3 scientific result or its observed provenance changed')
    if source['status']!='cooperative_geometry_passed':raise ValueError('Require a passed A3 source')
    manifest=json.loads((args.prepared_dir/'manifest.json').read_text())
    shapes={}
    for name in ('searchable','task_boundary','hazards'):
        path=args.prepared_dir/(name+'.geojson')
        if sha(path)!=manifest['derived_files_sha256'][path.name]:raise ValueError('Changed A1 shape: '+name)
        shapes[name]=json.loads(path.read_text())['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False)
    (args.output_dir/'config.json').write_bytes(args.config.read_bytes())
    hull=convex_hull(shapes['task_boundary']['coordinates'][0])
    original_hull=hull
    centre=[sum(p[j]for p in hull)/len(hull)for j in (0,1)]
    hull=[tuple(centre[j]+(1+cfg['support_hull_relative_expansion'])*(p[j]-centre[j])for j in (0,1))for p in hull]
    hull_shape={'type':'Polygon','coordinates':[[list(p)for p in hull+[hull[0]]]]}
    sweep=[]
    for radius in cfg['minimum_radius_sweep_m']:
        records=[]
        for name in ('uuv_1','uuv_2'):
            legs=source['platforms'][name]['legs']
            for i,leg in enumerate(legs):
                if not leg['scanning']:continue
                a,b=leg['points'];heading=math.atan2(b[1]-a[1],b[0]-a[0])
                for side,point,yaw,needed in [('entry',a,heading+math.pi,i>0),('exit',b,heading,i<len(legs)-1)]:
                    if not needed:continue
                    witnesses=forward_turn_witnesses(point,yaw,hull,radius)
                    records.append({'platform':name,'task_id':leg['task_id'],'source_leg_index':i,
                                    'side':side,'point':point,'heading_rad':yaw,'witnesses':witnesses})
        sweep.append({'minimum_radius_m':radius,'mandatory_scan_pose_endpoints_checked':len(records),
                      'certified_infeasible_endpoint_count':sum(bool(r['witnesses'])for r in records),
                      'failed_task_count':len({r['task_id']for r in records if r['witnesses']}),
                      'by_platform':{name:sum(bool(r['witnesses'])for r in records if r['platform']==name)
                                     for name in ('uuv_1','uuv_2')},'endpoint_records':records})
    corners={}
    for name,platform in source['platforms'].items():
        points=platform['path'];angles=[]
        for a,b,c in zip(points,points[1:],points[2:]):
            before=math.atan2(b[1]-a[1],b[0]-a[0]);after=math.atan2(c[1]-b[1],c[0]-b[0])
            delta=abs(math.remainder(after-before,2*math.pi))
            if delta>1e-8:angles.append(math.degrees(delta))
        corners[name]={'heading_discontinuity_count':len(angles),'maximum_heading_jump_deg':max(angles,default=0)}
    with Geos()as geos:
        if not geos.covers(hull_shape,shapes['searchable']):raise ValueError('Candidate is not within certified halfplanes')
        patch=None;rejected=0
        for name in ('uuv_1','uuv_2'):
            regular=[leg for leg in source['platforms'][name]['legs']if leg['kind']=='sweep']
            for a in regular:
                for b in regular:
                    radius=cfg['local_headland_radius_m'];length=cfg['local_segment_length_m']
                    ay,by=a['points'][0][1],b['points'][0][1]
                    if abs(by-ay-cfg['local_scan_spacing_m'])>1e-8 or by-ay<2*radius:continue
                    direction=1 if a['points'][1][0]>a['points'][0][0]else -1
                    if (b['points'][1][0]-b['points'][0][0])*direction>=0:continue
                    lo=max(min(p[0]for p in a['points']),min(p[0]for p in b['points']))
                    hi=min(max(p[0]for p in a['points']),max(p[0]for p in b['points']))
                    x=(hi-radius-cfg['local_headland_extra_inset_m']if direction==1
                       else lo+radius+cfg['local_headland_extra_inset_m'])
                    if (direction==1 and x-length<lo)or(direction==-1 and x+length>hi):continue
                    original_heading=0.0 if direction==1 else math.pi
                    certificate=forward_turn_witnesses(a['points'][-1],original_heading,hull,cfg['nominal_uuv_minimum_radius_m'])
                    if not certificate:continue
                    pieces=[line((x-direction*length,ay),(x,ay))]
                    if abs(by-ay-2*radius)<1e-8:
                        pieces.append(Piece('arc',pose=(x,ay,original_heading),length=math.pi*radius,curvature=direction/radius))
                    else:
                        pieces.extend([Piece('arc',pose=(x,ay,original_heading),length=math.pi*radius/2,curvature=direction/radius),
                                       line((x+direction*radius,ay+radius),(x+direction*radius,by-radius)),
                                       Piece('arc',pose=(x+direction*radius,by-radius,math.pi/2),length=math.pi*radius/2,curvature=direction/radius)])
                    pieces.append(line((x,by),(x-direction*length,by)))
                    points,sagitta=sample_enclosure(pieces,cfg['maximum_arc_sagitta_m'])
                    chord={'type':'LineString','coordinates':points}
                    # Compensate GEOS inscribed round caps so the enclosure
                    # contains the complete analytic curve, including arc bows.
                    buffer_radius=(sagitta+1e-5)/math.cos(math.pi/(4*cfg['footprint_quadrant_segments']))
                    envelope=geos.operation('buffer',chord,distance=buffer_radius,
                                            quadrant_segments=cfg['footprint_quadrant_segments'])
                    inside=geos.covers(shapes['searchable'],envelope)
                    clearance=geos.distance(shapes['hazards'],chord)-sagitta
                    if not inside or clearance<50:rejected+=1;continue
                    patch={'platform':name,'turn_radius_m':radius,'task_ids':[a['task_id'],b['task_id']],
                           'source_scans':[a,b],'new_scan_end':[x,ay],'new_next_scan_start':[x,by],
                           'retreat_distances_m':[math.dist(a['points'][-1],(x,ay)),math.dist(b['points'][0],(x,by))],
                           'pieces':[asdict(p)for p in pieces],'sampled_points':points,
                           'maximum_chord_sagitta_m':sagitta,'enclosure_buffer_radius_m':buffer_radius,
                           'whole_curve_enclosure_inside_candidate':inside,
                           'certified_original_hazard_clearance_lower_bound_m':clearance,
                           'source_exit_infeasibility_witnesses':certificate,
                           'profile':stopping_profile(pieces,cfg['cruise_mps'],cfg['tangential_accel_mps2'],cfg['lateral_accel_mps2']),
                           'interpretation':'Local turn plus100m scan on either side only; not the complete A3 route'}
                    (args.output_dir/'local_curve_enclosure.geojson').write_text(json.dumps(envelope)+'\n')
                    break
                if patch:break
            if patch:break
        if patch is None:raise ValueError('No certified local correction in the declared pair search')
        target=shapes['searchable'];target_area=geos.measure(target)['area_m2']
        baseline=[];modified=[]
        for task in source['tasks']:
            points=[list(p)for p in task['points']]
            # Use the executed direction, not the original A2 stored orientation.
            executed=next(leg for platform in source['platforms'].values()for leg in platform['legs']
                          if leg.get('task_id')==task['task_id'])
            points=[list(p)for p in executed['points']]
            def footprint(vertices):
                buffer=geos.operation('buffer',{'type':'LineString','coordinates':vertices},
                                      distance=cfg['sensor_radius_m'],quadrant_segments=cfg['footprint_quadrant_segments'])
                return geos.operation('intersection',buffer,target)
            baseline.append(footprint(points))
            if task['task_id']==patch['task_ids'][0]:points[-1]=patch['new_scan_end']
            if task['task_id']==patch['task_ids'][1]:points[0]=patch['new_next_scan_start']
            modified.append(footprint(points))
        original=geos.operation('union',{'type':'GeometryCollection','geometries':baseline})
        changed=geos.operation('union',{'type':'GeometryCollection','geometries':modified})
        missed=geos.operation('difference',target,changed)
        turn_points,turn_sagitta=sample_enclosure(pieces[1:-1],cfg['maximum_arc_sagitta_m'])
        # Hausdorff chord error <= sagitta: a (sensor_radius-sagitta) chord
        # buffer is a conservative subset of the true ideal curved footprint.
        turn_buffer=geos.operation('buffer',{'type':'LineString','coordinates':turn_points},
                                  distance=cfg['sensor_radius_m']-turn_sagitta,
                                  quadrant_segments=cfg['footprint_quadrant_segments'])
        turn_footprint=geos.operation('intersection',turn_buffer,target)
        turn_on=geos.operation('union',{'type':'GeometryCollection','geometries':[changed,turn_footprint]})
        turn_on_area=geos.measure(turn_on)['area_m2']
        upper_radius=(cfg['sensor_radius_m']+turn_sagitta)/math.cos(math.pi/(4*cfg['footprint_quadrant_segments']))
        turn_outer=geos.operation('buffer',{'type':'LineString','coordinates':turn_points},distance=upper_radius,
                                  quadrant_segments=cfg['footprint_quadrant_segments'])
        turn_outer=geos.operation('intersection',turn_outer,target)
        turn_on_upper=geos.operation('union',{'type':'GeometryCollection','geometries':[changed,turn_outer]})
        original_area=geos.measure(original)['area_m2'];changed_area=geos.measure(changed)['area_m2']
        coverage={'target_area_m2':target_area,'baseline_recomputed_covered_area_m2':original_area,
                  'baseline_area_error_vs_A3_m2':abs(original_area-source['coverage']['covered_area_m2']),
                  'after_one_patch_covered_area_m2':changed_area,'after_one_patch_coverage_ratio':changed_area/target_area,
                  'after_one_patch_missed_area_m2':geos.measure(missed)['area_m2'],
                  'coverage_loss_m2':original_area-changed_area,
                  'turn_connector_scanning_on_coverage_ratio_lower_bound':turn_on_area/target_area,
                  'turn_connector_scanning_on_coverage_ratio_upper_bound':geos.measure(turn_on_upper)['area_m2']/target_area,
                  'turn_connector_scanning_on_missed_area_upper_bound_m2':target_area-turn_on_area,
                  'turn_connector_footprint_chord_compensation_m':turn_sagitta,
                  'semantics':'Counterfactual footprint change for exactly two task endpoints; other84 footprints unchanged. Primary case turns scanning OFF; separately labelled ideal connector-scanning-ON sensitivity. Neither is a complete route or a physical sensor validation.'}
        if coverage['baseline_area_error_vs_A3_m2']>1e-3:raise ValueError('Baseline coverage failed independent reconstruction')
        for filename,geometry in [('single_patch_covered.geojson',changed),('single_patch_missed.geojson',missed),('turn_scanning_on_covered.geojson',turn_on),('turn_scanning_on_upper_covered.geojson',turn_on_upper)]:
            (args.output_dir/filename).write_text(json.dumps(geometry)+'\n')
    result={'stage':'A4','status':'workspace_audit_and_local_patch_completed',
            'full_mission_motion_feasible':False,'full_mission_retiming_released':False,
            'full_mission_reason':'Mandatory unchanged scan endpoint violates a containing-halfplane curvature bound; a local patch does not repair the full mission',
            'target_geometry_sha256':cfg['target_candidate_sha256'],'support_hull':hull,'original_task_support_hull':original_hull,
            'radius_sweep':sweep,'original_heading_discontinuities':corners,
            'local_patch':patch,'local_candidates_rejected':rejected,'counterfactual_coverage':coverage,
            'source_A3_unchanged':True,'limitations':cfg['scope_limitations']}
    if not next(r['certified_infeasible_endpoint_count']for r in sweep if r['minimum_radius_m']==cfg['nominal_uuv_minimum_radius_m']):
        raise ValueError('No nominal-radius certificate found; full-mission status needs review')
    (args.output_dir/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    profile=patch['profile'];sample_times={0.0,profile['duration_s']}
    sample_times.update(i*.5 for i in range(math.ceil(profile['duration_s']/.5)))
    for segment in profile['segments']:
        sample_times.update([segment['t0_s'],segment['t1_s'],segment['t0_s']+segment['ramp_duration_s'],segment['t1_s']-segment['ramp_duration_s']])
    local_pieces=[Piece(**p)for p in patch['pieces']]
    with(args.output_dir/'local_motion.csv').open('w')as stream:
        writer=csv.writer(stream);writer.writerow(['time_s','x_m','y_m','heading_rad','speed_mps','curvature_per_m','piece_index'])
        for t in sorted(sample_times):
            state=profile_state(profile,t);piece=local_pieces[state['piece_index']]
            pose=piece.evaluate(state['piece_distance_m']/piece.length)
            writer.writerow([t,*pose[:3],state['speed_mps'],pose[3],state['piece_index']])
    inputs=[args.config,args.a3_run_dir/'result.json',args.a3_run_dir/'provenance.json',
            *[args.prepared_dir/(n+'.geojson')for n in shapes],args.prepared_dir/'manifest.json']
    code=[Path(__file__),ROOT/'src/berthing_planner/motion_feasibility.py',ROOT/'src/berthing_planner/curves.py',ROOT/'scripts/chart_geometry.py']
    snap=args.output_dir/'code_snapshot';snap.mkdir()
    for path in code:(snap/path.name).write_bytes(path.read_bytes())
    (args.output_dir/'provenance.json').write_text(json.dumps({'recorded_utc':datetime.now(timezone.utc).isoformat(),
        'input_and_code_sha256':{str(p):sha(p)for p in inputs+code},'result_sha256':sha(args.output_dir/'result.json'),
        'local_motion_csv_sha256':sha(args.output_dir/'local_motion.csv'),
        'geometry_sha256':{p.name:sha(p)for p in args.output_dir.glob('*.geojson')}},indent=2)+'\n')
    print(json.dumps({'radius_sweep':[{k:v for k,v in r.items()if k!='endpoint_records'}for r in sweep],
                      'local_tasks':patch['task_ids'],'local_duration_s':patch['profile']['duration_s'],
                      'counterfactual_coverage':coverage},indent=2))


if __name__=='__main__':main()
