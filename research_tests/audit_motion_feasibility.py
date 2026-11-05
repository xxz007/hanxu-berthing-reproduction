"""Independent A4 halfplane, analytic-arc, clock and raw closed-grid audit.

No production motion, curve evaluation or speed-law imports. Raster slabs are
inflated by the certified arc-to-chord deviation, so sampled chord clearance
also certifies the intervening analytic arcs.
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from berthing_planner.validation.curves import piece_points, speed_checks

import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np






def inflated_grid_hits(a,b,blocked,bounds,padding):
    a,b=np.asarray(a),np.asarray(b);ny,nx=blocked.shape
    dx,dy=(bounds[2]-bounds[0])/nx,(bounds[3]-bounds[1])/ny
    low,high=np.minimum(a,b)-padding,np.maximum(a,b)+padding
    ilo=max(0,math.ceil((low[0]-bounds[0])/dx)-1);ihi=min(nx-1,math.floor((high[0]-bounds[0])/dx))
    jlo=max(0,math.ceil((low[1]-bounds[1])/dy)-1);jhi=min(ny-1,math.floor((high[1]-bounds[1])/dy))
    jj,ii=np.nonzero(blocked[jlo:jhi+1,ilo:ihi+1]);jj+=jlo;ii+=ilo
    lower=np.column_stack((bounds[0]+ii*dx,bounds[1]+jj*dy))-padding
    upper=lower+np.array([dx,dy])+2*padding
    enter=np.zeros(len(ii));leave=np.ones(len(ii));valid=np.ones(len(ii),dtype=bool)
    for axis in (0,1):
        delta=b[axis]-a[axis]
        if abs(delta)<1e-14:valid&=(lower[:,axis]<=a[axis])&(a[axis]<=upper[:,axis])
        else:
            t0=(lower[:,axis]-a[axis])/delta;t1=(upper[:,axis]-a[axis])/delta
            enter=np.maximum(enter,np.minimum(t0,t1));leave=np.minimum(leave,np.maximum(t0,t1))
    return int(np.sum(valid&(enter<=leave)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--a3-run-dir',type=Path,required=True)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--grid-dir',type=Path,required=True)
    parser.add_argument('--json-out',type=Path,required=True)
    args=parser.parse_args()
    x=json.loads((args.run_dir/'result.json').read_text());cfg=json.loads((args.run_dir/'config.json').read_text())
    source=json.loads((args.a3_run_dir/'result.json').read_text());grid=json.loads((args.grid_dir/'grid.json').read_text())
    provenance=json.loads((args.run_dir/'provenance.json').read_text())
    scientific={k:v for k,v in source.items()if k!='total_computation_seconds'}
    scientific_hash=hashlib.sha256(json.dumps(scientific,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    hashes=(hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()==provenance['result_sha256']
            and scientific_hash==cfg['source_a3_scientific_result_sha256']
            and hashlib.sha256((args.prepared_dir/'searchable.geojson').read_bytes()).hexdigest()==cfg['target_candidate_sha256'])
    hashes&=hashlib.sha256((args.grid_dir/'grid.json').read_bytes()).hexdigest()==cfg['original_grid_sha256']
    for name in ('hazards','task_boundary'):
        hashes&=hashlib.sha256((args.prepared_dir/(name+'.geojson')).read_bytes()).hexdigest()==cfg['a1_'+name+'_sha256']
    for name,digest in provenance['geometry_sha256'].items():
        hashes&=hashlib.sha256((args.run_dir/name).read_bytes()).hexdigest()==digest
    hashes&=hashlib.sha256((args.run_dir/'local_motion.csv').read_bytes()).hexdigest()==provenance['local_motion_csv_sha256']
    hull=np.asarray(x['support_hull']);edges=np.roll(hull,-1,axis=0)-hull
    normal=np.column_stack((edges[:,1],-edges[:,0]));normal/=np.linalg.norm(normal,axis=1)[:,None]
    constants=np.sum(normal*hull,axis=1)
    target=json.loads((args.prepared_dir/'searchable.geojson').read_text())['features'][0]['geometry']
    def vertices(v):
        if len(v)==2 and isinstance(v[0],(int,float)):yield v
        else:
            for q in v:yield from vertices(q)
    pts=np.asarray(list(vertices(target['coordinates'])))
    containment=float(np.max(pts@normal.T-constants))
    endpoints=[]
    for platform in ('uuv_1','uuv_2'):
        legs=source['platforms'][platform]['legs']
        for i,leg in enumerate(legs):
            if not leg['scanning']:continue
            a,b=np.asarray(leg['points']);u=(b-a)/np.linalg.norm(b-a)
            if i>0:endpoints.append((platform,leg['task_id'],'entry',a,-u))
            if i<len(legs)-1:endpoints.append((platform,leg['task_id'],'exit',b,u))
    counts=[];witness_errors=[]
    for record in x['radius_sweep']:
        radius=record['minimum_radius_m'];failures=0
        for platform,task,side,p,u in endpoints:
            dot=normal@u
            clearance=constants-normal@p
            necessary=radius*(1-np.sqrt(np.maximum(0,1-np.clip(dot,-1,1)**2)))
            failed=(dot>1e-10)&(clearance<necessary-1e-7)
            failures+=bool(np.any(failed))
            original=next(r for r in record['endpoint_records']if(r['platform'],r['task_id'],r['side'])==(platform,task,side))
            if bool(original['witnesses'])!=bool(np.any(failed)):witness_errors.append([radius,task,side])
            for w in original['witnesses']:
                j=w['supporting_edge_index']
                if not failed[j]or abs(w['available_outward_distance_m']-clearance[j])>1e-7:witness_errors.append([radius,task,'incorrect edge'])
        counts.append({'minimum_radius_m':radius,'independent_failed_endpoints':failures})
        if failures!=record['certified_infeasible_endpoint_count']or len(endpoints)!=record['mandatory_scan_pose_endpoints_checked']:
            witness_errors.append([radius,'count mismatch'])
    patch=x['local_patch'];pieces=patch['pieces'];join_error=0;heading_error=0;radius_pass=True
    fine=[]
    for i,p in enumerate(pieces):
        n=max(1,math.ceil(p['length']/.5));positions=piece_points(p,np.linspace(0,1,n+1))
        fine.extend(positions if not fine else positions[1:])
        radius_pass&=abs(p['curvature'])<=1/cfg['nominal_uuv_minimum_radius_m']+1e-12
        if i:
            prior=pieces[i-1];end=piece_points(prior,[1])[0]
            join_error=max(join_error,float(np.linalg.norm(end-positions[0])))
            heading_error=max(heading_error,abs(math.remainder(prior['pose'][2]+prior['curvature']*prior['length']-p['pose'][2],2*math.pi)))
    blocked=np.asarray(grid['rows_bottom_to_top'],dtype=bool)
    # Use the saved chord with its error bound; cross-check the full analytic
    # curve independently at 0.5m spacing against that saved centreline.
    chords=np.asarray(patch['sampled_points']);bound=patch['maximum_chord_sagitta_m']
    fine=np.asarray(fine);distance2=np.full(len(fine),np.inf)
    for a,b in zip(chords,chords[1:]):
        d=b-a;fraction=np.clip((fine-a)@d/(d@d),0,1);delta=fine-(a+fraction[:,None]*d)
        distance2=np.minimum(distance2,np.sum(delta*delta,axis=1))
    max_chord_deviation=float(np.sqrt(np.max(distance2)))
    collision_hits=sum(inflated_grid_hits(a,b,blocked,grid['bounds'],bound+1e-6)for a,b in zip(chords,chords[1:]))
    speeds=speed_checks(patch['profile'],pieces,cfg)
    target_area=json.loads((args.prepared_dir/'summary.json').read_text())['searchable_area_m2']
    unchanged_target=abs(x['counterfactual_coverage']['target_area_m2']-target_area)<1e-6
    # Independent pointwise coverage uses original target samples and ideal
    # straight footprints. Exact native area and sampling remain distinct.
    from audit_open_chart_stage_a import inside
    from audit_open_chart_coverage import active_pass_counts
    xx,yy=np.meshgrid(np.arange(505,5120,10),np.arange(705,4400,10));points=np.column_stack((xx.ravel(),yy.ravel()))
    inside_target,on_boundary=inside(target,points);points=points[inside_target&~on_boundary]
    legs=[dict(leg,points=[list(p)for p in leg['points']])for platform in source['platforms'].values()for leg in platform['legs']if leg['scanning']]
    before=active_pass_counts(points,legs,cfg['sensor_radius_m'])>0
    for leg in legs:
        if leg['task_id']==patch['task_ids'][0]:leg['points'][-1]=patch['new_scan_end']
        if leg['task_id']==patch['task_ids'][1]:leg['points'][0]=patch['new_next_scan_start']
    after=active_pass_counts(points,legs,cfg['sensor_radius_m'])>0
    ideal_turn=after.copy()
    from audit_open_chart_coverage import point_segment_distance_squared
    for piece in pieces[1:-1]:
        if piece['curvature']==0:
            ends=piece_points(piece,[0,1]);distance=np.sqrt(point_segment_distance_squared(points,*ends))
        else:
            k=piece['curvature'];r=1/abs(k);px,py,yaw=piece['pose']
            centre=np.array([px-math.sin(yaw)/k,py+math.cos(yaw)/k]);delta=points-centre
            angle=np.arctan2(delta[:,1],delta[:,0]);start=yaw-math.copysign(math.pi/2,k)
            progress=np.mod(math.copysign(1,k)*(angle-start),2*math.pi)
            ends=piece_points(piece,[0,1]);end_distance=np.minimum(np.linalg.norm(points-ends[0],axis=1),np.linalg.norm(points-ends[1],axis=1))
            distance=np.where(progress<=piece['length']/r+1e-12,abs(np.linalg.norm(delta,axis=1)-r),end_distance)
        ideal_turn|=distance<=cfg['sensor_radius_m']
    sampled_loss=int(np.sum(before&~after))*100
    area_loss=x['counterfactual_coverage']['coverage_loss_m2']
    coverage=x['counterfactual_coverage']
    bracket=(0<=coverage['after_one_patch_coverage_ratio']<=coverage['turn_connector_scanning_on_coverage_ratio_lower_bound']
             <=coverage['turn_connector_scanning_on_coverage_ratio_upper_bound']<=1)
    area_direction_pass=sampled_loss>=0 and area_loss>=-1e-5 and abs(sampled_loss-area_loss)<1000
    passed=bool(hashes and containment<=1e-8 and not witness_errors and collision_hits==0 and radius_pass
                and join_error<1e-6 and heading_error<1e-8 and max_chord_deviation<=bound+1e-7
                and 0<=bound<=cfg['maximum_arc_sagitta_m']+1e-10
                and not speeds and unchanged_target and area_direction_pass and bracket
                and x['full_mission_motion_feasible']is False and x['full_mission_retiming_released']is False)
    report={'passed':passed,'source_hashes_match':bool(hashes),'maximum_target_outside_halfplanes_m':containment,
            'independent_radius_sweep':counts,'witness_errors':witness_errors,
            'local_position_join_error_m':join_error,'local_heading_join_error_rad':heading_error,
            'local_curvature_bound_passed':bool(radius_pass),'original_grid_inflated_closed_cell_hits':collision_hits,
            'independent_maximum_chord_deviation_m':max_chord_deviation,'declared_arc_chord_error_bound_m':bound,
            'speed_profile_errors':speeds,'original_target_area_unchanged':unchanged_target,
            'sampled_scan_off_loss_m2':sampled_loss,'native_scan_off_loss_m2':area_loss,
            'independent_ideal_turn_scan_on_sample_coverage_ratio':float(np.mean(ideal_turn)),
            'native_turn_scan_on_area_coverage_lower_bound':x['counterfactual_coverage']['turn_connector_scanning_on_coverage_ratio_lower_bound'],
            'turn_scan_on_lower_upper_bracket_consistent':bracket,
            'sampling_note':'10m strict-interior samples; sampling does not certify absence of tiny residuals. Native GEOS area is separately retained.',
            'full_mission_retiming_correctly_withheld':not x['full_mission_retiming_released']}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if not passed:raise SystemExit(1)


if __name__=='__main__':main()
