"""A6 finite joint scan geometry/order trial with exact chart-envelope checks.

All86 source task IDs and the full original target survive. Feasible complete
routes get new footprints; failures cannot inherit A3 coverage or mission time.
"""
import argparse
from dataclasses import asdict
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece,line
from berthing_planner.dubins import candidates
from berthing_planner.headland import reachable_boxes
from berthing_planner.joint_coverage import expanded_scan_variants,task_order_candidates,plan_variant_order
from berthing_planner.motion_feasibility import stopping_profile
from berthing_planner.maneuver_search import forward_pose_search
from navigation_geometry import NavigationGeometry
from run_motion_feasibility import sample_enclosure
from scan_footprints import straight_footprint_bounds


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a3-run-dir','a5-run-dir','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/joint_coverage_a6.json')
    args=parser.parse_args();cfg=json.loads(args.config.read_text());started=time.perf_counter()
    srcpath=args.a3_run_dir/'result.json';source=json.loads(srcpath.read_text());science={k:v for k,v in source.items()if k!='total_computation_seconds'}
    if hashlib.sha256(json.dumps(science,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()!=cfg['source_a3_scientific_result_sha256']:raise ValueError('Changed A3 science')
    shapes={};files=[args.config,srcpath,args.navigation_dir/'navigation.geojson'];base_variants={}
    for name,digest in cfg['a5_variant_files_sha256'].items():
        p=args.a5_run_dir/(name+'_variants.json');files.append(p)
        if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise ValueError('Changed A5 candidate snapshot')
        base_variants[name]=json.loads(p.read_text())
    for name in ('searchable','hazards'):
        p=args.prepared_dir/(name+'.geojson');files.append(p)
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg['A1_geometry_sha256'][name]:raise ValueError('Changed A1 geometry')
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    navpath=args.navigation_dir/'navigation.geojson'
    if hashlib.sha256(navpath.read_bytes()).hexdigest()!=cfg['expected_navigation_sha256']:raise ValueError('Changed safe navigation')
    nav=json.loads(navpath.read_text())['features'][0]['geometry'];target=shapes['searchable']
    args.output_dir.mkdir(parents=True,exist_ok=False);(args.output_dir/'config.json').write_bytes(args.config.read_bytes())
    plans={};diagnostics={};cache={};native_queries=0
    with NavigationGeometry(nav,shapes['hazards'])as n:
        g=n.geos
        def motion_edge(start,goal,radius):
            nonlocal native_queries
            key=(tuple(start),tuple(goal),radius)
            if key in cache:return cache[key]
            for candidate in candidates(start,goal,radius):
                if not candidate['pieces']:
                    if n.covers({'type':'Point','coordinates':start[:2]}):cache[key]={'pieces':[],'length_m':0.,'word':'identity'};return cache[key]
                    continue
                xy,error=sample_enclosure(candidate['pieces'],cfg['maximum_arc_sagitta_m']);chord={'type':'LineString','coordinates':xy}
                envelope=g.operation('buffer',chord,distance=(error+1e-5)/math.cos(math.pi/120),quadrant_segments=30);native_queries+=1
                if not n.covers(envelope):continue
                clearance=n.distance(chord,to_hazards=True)-error
                if clearance<cfg['minimum_original_hazard_clearance_m']:continue
                cache[key]={**candidate,'clearance_lower_bound_m':clearance};return cache[key]
            cache[key]=None;return None
        def footprint(points,radius):
            return g.operation('buffer',{'type':'LineString','coordinates':points},distance=radius,quadrant_segments=cfg['footprint_quadrant_segments'])
        for name in ('uuv_1','uuv_2'):
            tasks=[t for t in source['platforms'][name]['legs']if t['scanning']];layers={};variant_notes=[]
            for task_index,task in enumerate(tasks):
                accepted=[];variants=expanded_scan_variants(task['points'],cfg['uuv_minimum_radius_m']);old=footprint(task['points'],cfg['sensor_radius_m']);loss_cache={}
                for v in variants:
                    if not n.covers({'type':'LineString','coordinates':v['points']}):continue
                    yaw=v['heading_rad'];a,b=v['points']
                    # A declared candidate filter: both directions must allow
                    # 30m of continued forward motion. This can exclude valid
                    # free terminal poses; it is not a global impossibility claim.
                    if any(n.distance(box)>1e-5 for pose in ((*a,yaw+math.pi),(*b,yaw))for _,box in reachable_boxes(pose,cfg['uuv_minimum_radius_m'],30.1)):continue
                    signature=tuple(sorted(tuple(round(x,8)for x in p)for p in v['points']))
                    if signature not in loss_cache:loss_cache[signature]=g.measure(g.operation('difference',old,footprint(v['points'],cfg['sensor_radius_m'])))['area_m2']
                    loss=loss_cache[signature];v.update({'task_id':task['task_id'],'original_points':task['points'],'unclipped_capsule_loss_m2':loss,
                           'node_cost':v['length_m']+cfg['coverage_loss_weight']*loss/(2*cfg['sensor_radius_m'])})
                    accepted.append(v)
                # Preserve angular diversity. The finite truncation is part
                # of the search definition, and is recorded per source task.
                groups={};short=task['length_m']<4*cfg['uuv_minimum_radius_m']
                for v in accepted:
                    bins=cfg['short_variant_heading_buckets']if short else 2
                    bucket=round((v['heading_rad']%(2*math.pi))*bins/(2*math.pi))%bins
                    groups.setdefault(bucket,[]).append(v)
                limit=cfg['maximum_variants_per_heading_bucket']if short else cfg['long_variants_per_direction']
                kept=[v for bucket in sorted(groups)for v in sorted(groups[bucket],key=lambda x:(x['node_cost'],x['kind']))[:limit]]
                mandatory=[]
                for original in base_variants[name][task_index]:
                    if original['task_id']!=task['task_id']:raise ValueError('A5 task/order mismatch')
                    for points in (original['points'],original['points'][::-1]):
                        yaw=math.atan2(points[1][1]-points[0][1],points[1][0]-points[0][0]);size=math.dist(*points)
                        loss=g.measure(g.operation('difference',old,footprint(points,cfg['sensor_radius_m'])))['area_m2']
                        mandatory.append({'task_id':task['task_id'],'points':points,'heading_rad':yaw,'length_m':size,'kind':'retained_A5_candidate',
                                          'original_points':task['points'],'unclipped_capsule_loss_m2':loss,'node_cost':size+cfg['coverage_loss_weight']*loss/(2*cfg['sensor_radius_m'])})
                unique={tuple(round(x,8)for p in v['points']for x in p):v for v in mandatory+kept};kept=sorted(unique.values(),key=lambda v:(v['node_cost'],v['kind']))
                layers[task['task_id']]=kept;variant_notes.append({'task_id':task['task_id'],'generated':len(variants),'admissible':len(accepted),'retained':len(kept)})
                print(name,task['task_id'],'variants',len(accepted),'kept',len(kept),flush=True)
            (args.output_dir/(name+'_variants.json')).write_text(json.dumps(layers,indent=2)+'\n')
            alternatives=[];attempts=[];oldlegs=source['platforms'][name]['legs'];spines={};shifted_spines={};search_records=[];search_pairs=set();search_counts={};search_solved_layers=set()
            for i,leg in enumerate(oldlegs):
                if leg['scanning']:continue
                key=(oldlegs[i-1]['task_id'],oldlegs[i+1]['task_id']);spines[key]=leg['points'];spines[key[::-1]]=leg['points'][::-1]
            box={'type':'Polygon','coordinates':[[[0,0],[5120,0],[5120,5120],[0,5120],[0,0]]]};blocked=g.operation('difference',box,nav)
            def direct(k,i,j,a,b):return motion_edge((*a['points'][1],a['heading_rad']),(*b['points'][0],b['heading_rad']),cfg['uuv_minimum_radius_m'])
            # Bounded one-pose detours are tried only for a disconnected layer.
            def fallback(k,i,j,a,b):
                start=(*a['points'][1],a['heading_rad']);goal=(*b['points'][0],b['heading_rad']);radius=cfg['uuv_minimum_radius_m']
                key=(a['task_id'],b['task_id'])
                if key not in shifted_spines:
                    shifted=[]
                    for p in spines.get(key,[])[1:-1]:
                        options=[p]+[[p[0]+d*math.cos(i*math.pi/4),p[1]+d*math.sin(i*math.pi/4)]for d in (radius,2*radius,3*radius)for i in range(8)]
                        free=[q for q in options if n.covers({'type':'Point','coordinates':q})]
                        if free:shifted.append(max(free,key=lambda q:g.distance({'type':'Point','coordinates':q},blocked)))
                    shifted_spines[key]=shifted
                waypoints=shifted_spines[key]
                if waypoints:
                    xy=[start[:2]]+waypoints+[goal[:2]]
                    poses=[start]+[(*p,math.atan2(xy[k+1][1]-xy[k-1][1],xy[k+1][0]-xy[k-1][0]))for k,p in enumerate(xy[1:-1],1)]+[goal]
                    links=[motion_edge(p,q,radius)for p,q in zip(poses,poses[1:])]
                    if all(link is not None for link in links):return {'pieces':[p for link in links for p in link['pieces']],'length_m':sum(link['length_m']for link in links),'word':'shifted_A3_spine'}
                mid=[(start[d]+goal[d])/2 for d in (0,1)];yaw=math.atan2(goal[1]-start[1],goal[0]-start[0])
                for distance in (2*radius,4*radius,8*radius):
                    for angle in (yaw+math.pi/2,yaw-math.pi/2):
                        p=[mid[0]+distance*math.cos(angle),mid[1]+distance*math.sin(angle)]
                        if not n.covers({'type':'Point','coordinates':p}):continue
                        for heading in (yaw,yaw+math.pi/2,yaw-math.pi/2):
                            first=motion_edge(start,(*p,heading),radius)
                            if first is None:continue
                            second=motion_edge((*p,heading),goal,radius)
                            if second is not None:return {'pieces':first['pieces']+second['pieces'],'length_m':first['length_m']+second['length_m'],'word':'finite_one_pose_detour'}
                return None
            def pose_connection(k,i,j,a,b):
                nonlocal native_queries
                start=(*a['points'][1],a['heading_rad']);goal=(*b['points'][0],b['heading_rad']);radius=cfg['uuv_minimum_radius_m']
                pair=tuple(round(z,7)for z in (*start,*goal));layer=(a['task_id'],b['task_id'])
                if layer in search_solved_layers:return None
                if pair in search_pairs or search_counts.get(layer,0)>=cfg['pose_search_max_calls_per_layer']:return None
                search_pairs.add(pair);search_counts[layer]=search_counts.get(layer,0)+1
                def clear(pp):
                    nonlocal native_queries
                    if not pp:return True
                    xy,error=sample_enclosure(pp,cfg['maximum_arc_sagitta_m']);native_queries+=1
                    envelope=g.operation('buffer',{'type':'LineString','coordinates':xy},distance=(error+1e-5)/math.cos(math.pi/120),quadrant_segments=30)
                    return n.covers(envelope)
                margin=cfg['pose_search_margin_m'];bounds=(max(0,min(start[0],goal[0])-margin),max(0,min(start[1],goal[1])-margin),min(5120,max(start[0],goal[0])+margin),min(5120,max(start[1],goal[1])+margin))
                result=forward_pose_search(start,goal,radius,clear,bounds=bounds,position_resolution=cfg['pose_search_position_resolution_m'],
                    heading_bins=cfg['pose_search_heading_bins'],straight_step=cfg['pose_search_straight_step_m'],max_expansions=cfg['pose_search_max_expansions'],
                    shot_every=cfg['pose_search_shot_every'],shot_distance=cfg['pose_search_shot_distance_m'])
                search_records.append({'prior_task_id':a['task_id'],'next_task_id':b['task_id'],'start_pose':start,'goal_pose':goal,**{key:value for key,value in result.items()if key!='pieces'}})
                if result['passed']:
                    search_solved_layers.add(layer);return result
                return None
            orders=task_order_candidates(tasks,cfg['uuv_minimum_radius_m']);priority=cfg['order_priority'][name]
            orders=sorted(orders,key=lambda item:priority.index(item[0])if item[0]in priority else len(priority)+orders.index(item))
            for order_name,ids in orders:
                plan=plan_variant_order([layers[t]for t in ids],[direct,fallback,pose_connection])
                attempts.append({'order':order_name,'task_ids':ids,'passed':plan['passed'],**{k:v for k,v in plan.items()if k in ('failed_layer','empty_layers','layers_reached')}})
                print(name,order_name,attempts[-1],flush=True)
                if not plan['passed']:continue
                selected=[layers[t][j]for t,j in zip(ids,plan['variant_indices'])];pieces=[];legs=[]
                for i,v in enumerate(selected):
                    if i:
                        edge=plan['edges'][i-1];pieces.extend(edge['pieces']);legs.append({'kind':'transit','scanning':False,'pieces':[asdict(p)for p in edge['pieces']],'length_m':edge['length_m'],'method':edge['word']})
                    p=line(*v['points']);pieces.append(p);legs.append({'kind':'scan','scanning':True,'task_id':v['task_id'],'points':v['points'],'pieces':[asdict(p)],'length_m':p.length})
                alternatives.append({'order':order_name,'task_ids':ids,'selected_scans':selected,'pieces':[asdict(p)for p in pieces],'legs':legs,'length_m':sum(p.length for p in pieces),
                                     'profile':stopping_profile(pieces,cfg['cruise_mps'][name],cfg['tangential_accel_mps2'],cfg['lateral_accel_mps2'])})
                break  # First complete order; no global order-optimality claim.
            diagnostics[name]={'task_variants':variant_notes,'order_attempts':attempts,'pose_searches':search_records};plans[name]=alternatives
            (args.output_dir/(name+'_alternatives.json')).write_text(json.dumps(alternatives,indent=2)+'\n')
        # USV A5 geometry is retained as a separately checked independent route.
        legs=source['platforms']['usv']['legs'];knots=[legs[0]['points'][0]]+[x['points'][-1]for x in legs]
        headings=[math.atan2(knots[min(i+1,len(knots)-1)][1]-knots[max(0,i-1)][1],knots[min(i+1,len(knots)-1)][0]-knots[max(0,i-1)][0])for i in range(len(knots))]
        links=[motion_edge((*a,headings[i]),(*b,headings[i+1]),cfg['usv_minimum_radius_m'])for i,(a,b)in enumerate(zip(knots,knots[1:]))]
        usv={}
        if all(x is not None for x in links):
            pp=[p for link in links for p in link['pieces']];usv={'pieces':[asdict(p)for p in pp],'length_m':sum(p.length for p in pp),'profile':stopping_profile(pp,cfg['cruise_mps']['usv'],cfg['tangential_accel_mps2'],cfg['lateral_accel_mps2'])}
        combinations=[];chosen=None;coverage={};target_area=g.measure(target)['area_m2']
        for first,second in itertools.product(plans['uuv_1'],plans['uuv_2']):
            scans=[footprint(v['points'],cfg['sensor_radius_m'])for plan in (first,second)for v in plan['selected_scans']]
            covered=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':scans}),target)
            ratio=g.measure(covered)['area_m2']/target_area;record={'orders':[first['order'],second['order']],'scan_only_coverage_ratio':ratio,'total_uuv_length_m':first['length_m']+second['length_m']};combinations.append(record)
            # Ignore sub-millimetre-square Boolean-operation roundoff when
            # comparing forward/reversed versions of identical footprints.
            if chosen is None or(round(ratio,10),-round(record['total_uuv_length_m'],6))>(round(coverage['scan_only_coverage_ratio'],10),-round(coverage['total_uuv_length_m'],6)):chosen=(first,second,covered);coverage=record
        platforms={};complete=chosen is not None and bool(usv)
        if complete:
            first,second,covered=chosen;platforms={'uuv_1':first,'uuv_2':second,'usv':usv};lower=[covered]
            upper_scans=[straight_footprint_bounds(g,v['points'],cfg['sensor_radius_m'],cfg['footprint_quadrant_segments'])[1]for plan in (first,second)for v in plan['selected_scans']]
            scan_hi=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':upper_scans}),target);upper=[scan_hi]
            for plan in (first,second):
                for leg in plan['legs']:
                    if leg['scanning']or not leg['pieces']:continue
                    xy,error=sample_enclosure([Piece(**p)for p in leg['pieces']],cfg['maximum_arc_sagitta_m']);shape={'type':'LineString','coordinates':xy}
                    lower.append(g.operation('buffer',shape,distance=cfg['sensor_radius_m']-error,quadrant_segments=30))
                    upper.append(g.operation('buffer',shape,distance=(cfg['sensor_radius_m']+error)/math.cos(math.pi/120),quadrant_segments=30))
            lo=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':lower}),target)
            hi=g.operation('intersection',g.operation('union',{'type':'GeometryCollection','geometries':upper}),target)
            coverage.update({'target_area_m2':target_area,'scan_only_coverage_upper_ratio':g.measure(scan_hi)['area_m2']/target_area,'scan_only_missed_area_m2':target_area-g.measure(covered)['area_m2'],
                             'transit_scan_ON_lower_ratio':g.measure(lo)['area_m2']/target_area,'transit_scan_ON_upper_ratio':g.measure(hi)['area_m2']/target_area})
            for filename,shape in [('scan_only_covered.geojson',covered),('scan_only_missed.geojson',g.operation('difference',target,covered)),('transit_scan_on_covered_lower.geojson',lo),('transit_scan_on_covered_upper.geojson',hi),('transit_scan_on_missed_upper.geojson',g.operation('difference',target,lo))]:
                (args.output_dir/filename).write_text(json.dumps(shape)+'\n')
    result={'stage':'A6','complete_platform_routes_found':complete,'platforms':platforms,'diagnostics':diagnostics,'route_combinations':combinations,'coverage':coverage,
            'coverage_target_sha256':cfg['A1_geometry_sha256']['searchable'],'navigation_sha256':cfg['expected_navigation_sha256'],
            'full_mission_retiming_released':False,'cooperative_separation_verified':False,'method':'Finite scan variants + configured task-order candidates + layered graph/Dubins/waypoints/bounded forward pose search; no global optimality or infeasibility claim.',
            'selection':'Configured finite order priorities (source or grouped short-cluster terminal visit); stop at first complete order per platform. Length plus local capsule-loss proxy inside tested graph; full-target coverage recomputed separately.',
            'node_cost_scope':'Unclipped original scan capsule-loss heuristic; final coverage recomputed on unchanged full target.',
            'candidate_filter_scope':'New variants require30m continued motion at both scan ends; every original A5 admissible candidate is separately retained. Finite heading buckets, not exhaustive.',
            'native_curve_queries':native_queries,'unique_pose_pairs':len(cache),'total_computation_seconds':time.perf_counter()-started}
    (args.output_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    code=[Path(__file__),ROOT/'scripts/navigation_geometry.py',ROOT/'scripts/chart_geometry.py',ROOT/'scripts/run_motion_feasibility.py',ROOT/'scripts/scan_footprints.py']+[ROOT/'src/berthing_planner'/f'{name}.py'for name in ('joint_coverage','headland','dubins','curves','motion_feasibility','maneuver_search')]
    derived=[p for p in args.output_dir.iterdir()if p.name not in ('result.json','config.json')]
    provenance={'input_sha256':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in files},'code_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in code},'result_sha256':hashlib.sha256((args.output_dir/'result.json').read_bytes()).hexdigest(),
                'derived_files_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in derived}}
    (args.output_dir/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n');print(json.dumps({'complete_platform_routes_found':complete,'coverage':coverage,'elapsed_s':result['total_computation_seconds']},indent=2))


if __name__=='__main__':main()
