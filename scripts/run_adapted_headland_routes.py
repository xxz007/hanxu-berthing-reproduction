"""Finite variant-graph trial: endpoint retreats/extensions and short through-scans.

Preserve every task ID and the full original coverage target. All adaptations,
missing tasks and failed connections remain explicit. No arbitrary skipped
task can turn a partial route into a reported complete mission.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece,line
from berthing_planner.dubins import candidates
from berthing_planner.headland import reachable_boxes,scan_variants,select_variant_chain
from berthing_planner.motion_feasibility import stopping_profile
from navigation_geometry import NavigationGeometry
from run_motion_feasibility import sample_enclosure


def rings(shape):
    if shape['type']=='Polygon':yield from shape['coordinates']
    elif shape['type']=='MultiPolygon':
        for p in shape['coordinates']:yield from p
    elif shape['type']=='GeometryCollection':
        for g in shape['geometries']:yield from rings(g)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a3-run-dir',type=Path,required=True)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--navigation-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/headland_navigation_a5.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args();cfg=json.loads(args.config.read_text());source=json.loads((args.a3_run_dir/'result.json').read_text())
    scientific={k:v for k,v in source.items()if k!='total_computation_seconds'}
    if hashlib.sha256(json.dumps(scientific,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()!=cfg['source_a3_scientific_result_sha256']:
        raise ValueError('A3 source scientific result changed')
    shapes={}
    for name in ('searchable','hazards'):
        p=args.prepared_dir/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg['A1_geometry_sha256'][name]:raise ValueError('A1 scientific geometry changed')
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    navpath=args.navigation_dir/'navigation.geojson';navsummary=json.loads((args.navigation_dir/'summary.json').read_text())
    if hashlib.sha256(navpath.read_bytes()).hexdigest()!=navsummary['navigation_sha256']:raise ValueError('Navigation changed')
    if navsummary['navigation_sha256']!=cfg['expected_navigation_sha256']:raise ValueError('Navigation does not match fixed A5 geometry')
    nav=json.loads(navpath.read_text())['features'][0]['geometry'];target=shapes['searchable'];hazards=shapes['hazards']
    args.output_dir.mkdir(parents=True,exist_ok=False);(args.output_dir/'config.json').write_bytes(args.config.read_bytes())
    edges=np.asarray([(a,b)for ring in rings(nav)for a,b in zip(ring,ring[1:])if a!=b]);boundary_a,boundary_b=edges[:,0,:],edges[:,1,:]
    vector=boundary_b-boundary_a;norm2=np.sum(vector*vector,axis=1)
    platforms={};diagnostics={}
    with NavigationGeometry(nav,hazards)as navigation:
        g=navigation.geos
        def point_shape(p):return {'type':'Point','coordinates':list(p)}
        def blocked_pose(pose,radius,length):
            return any(navigation.distance(box)>1e-5 for _,box in reachable_boxes(pose,radius,length))
        def footprint(points,radius):
            return g.operation('intersection',g.operation('buffer',{'type':'LineString','coordinates':points},distance=radius,quadrant_segments=30),target)
        def motion_edge(start,goal,radius):
            for candidate in candidates(start,goal,radius):
                if not candidate['pieces']:
                    return {'pieces':[],'length_m':candidate['length_m'],'word':candidate['word'],'clearance_lower_bound_m':None}
                points,error=sample_enclosure(candidate['pieces'],cfg['maximum_arc_sagitta_m'])
                chord={'type':'LineString','coordinates':points}
                envelope=g.operation('buffer',chord,distance=(error+1e-5)/math.cos(math.pi/120),quadrant_segments=30)
                if not navigation.covers(envelope):continue
                clearance=navigation.distance(chord,to_hazards=True)-error
                if clearance>=50:
                    return {'pieces':candidate['pieces'],'length_m':candidate['length_m'],'word':candidate['word'],
                            'clearance_lower_bound_m':clearance}
            return None
        for name in ('uuv_1','uuv_2'):
            tasks=[leg for leg in source['platforms'][name]['legs']if leg['scanning']]
            radius=cfg['uuv_minimum_radius_m'];layers=[];raw_counts=[];pruned=0
            minimum_scan=min(min(t['length_m'],.2*t['length_m'])if t['length_m']>=4*radius else t['length_m']for t in tasks)
            for index,task in enumerate(tasks):
                a,b=task['points'];yaw=math.atan2(b[1]-a[1],b[0]-a[0]);past=minimum_scan*index;future=minimum_scan*(len(tasks)-index-1)
                bad_entry=blocked_pose((*a,yaw+math.pi),radius,past);bad_exit=blocked_pose((*b,yaw),radius,future)
                centre=(np.asarray(a)+b)/2;fraction=np.clip(np.sum((centre-boundary_a)*vector,axis=1)/norm2,0,1)
                delta=centre-(boundary_a+fraction[:,None]*vector);nearest=int(np.argmin(np.sum(delta*delta,axis=1)))
                tangent=math.atan2(vector[nearest,1],vector[nearest,0])
                variants=scan_variants(task['points'],radius,tangent,bad_entry,bad_exit);raw_counts.append(len(variants));accepted=[]
                old=footprint(task['points'],cfg['sensor_radius_m'])
                for v in variants:
                    if not navigation.covers({'type':'LineString','coordinates':v['points']}):pruned+=1;continue
                    entry=(*v['points'][0],v['heading_rad']+math.pi);exit=(*v['points'][1],v['heading_rad'])
                    if blocked_pose(entry,radius,past)or blocked_pose(exit,radius,future):pruned+=1;continue
                    changed=footprint(v['points'],cfg['sensor_radius_m']);loss=g.measure(g.operation('difference',old,changed))['area_m2']
                    v.update({'task_id':task['task_id'],'original_points':task['points'],'lost_original_footprint_area_m2':loss,
                              'node_cost':v['length_m']+20*loss/(2*cfg['sensor_radius_m'])})
                    accepted.append(v)
                layers.append(accepted)
            print(name,'variant counts',[len(v)for v in layers],flush=True)
            tested=0;fallback_tested=0
            def direct(layer,i,j,a,b):
                nonlocal tested
                tested+=1
                return motion_edge((*a['points'][1],a['heading_rad']),(*b['points'][0],b['heading_rad']),radius)
            # Rare topological detours: use the original collision-free A3
            # connector spine, with waypoints shifted toward wider free water.
            old_transits={}
            oldlegs=source['platforms'][name]['legs']
            for i,leg in enumerate(oldlegs):
                if not leg['scanning']:old_transits[(oldlegs[i-1]['task_id'],oldlegs[i+1]['task_id'])]=leg['points']
            box={'type':'Polygon','coordinates':[[[0,0],[5120,0],[5120,5120],[0,5120],[0,0]]]}
            blocked=g.operation('difference',box,nav)
            def detour(layer,i,j,a,b):
                nonlocal fallback_tested
                fallback_tested+=1
                start=(*a['points'][1],a['heading_rad']);goal=(*b['points'][0],b['heading_rad'])
                spine=old_transits.get((a['task_id'],b['task_id']),[])
                waypoints=[]
                for p in spine[1:-1]:
                    choices=[p]+[[p[0]+distance*math.cos(angle),p[1]+distance*math.sin(angle)]for distance in (radius,2*radius)for angle in np.linspace(0,2*math.pi,8,endpoint=False)]
                    valid=[q for q in choices if navigation.covers(point_shape(q))]
                    if valid:waypoints.append(max(valid,key=lambda q:g.distance(point_shape(q),blocked)))
                poses=[start]+[(*p,math.atan2((waypoints[k+1]if k+1<len(waypoints)else goal)[1]-(waypoints[k-1]if k else start)[1],
                                             (waypoints[k+1]if k+1<len(waypoints)else goal)[0]-(waypoints[k-1]if k else start)[0]))for k,p in enumerate(waypoints)]+[goal]
                if len(poses)>2:
                    links=[motion_edge(p,q,radius)for p,q in zip(poses,poses[1:])]
                    if all(links):return {'pieces':[p for e in links for p in e['pieces']],'length_m':sum(e['length_m']for e in links),'word':'shifted_A3_spine'}
                # One intermediate pose is a bounded fallback, not a claim of
                # complete search through all free-water homotopy classes.
                mid=[(start[k]+goal[k])/2 for k in(0,1)];direction=math.atan2(goal[1]-start[1],goal[0]-start[0])
                points=waypoints[:4]+[[mid[0]+distance*math.cos(angle),mid[1]+distance*math.sin(angle)]for distance in (2*radius,4*radius)for angle in (direction+math.pi/2,direction-math.pi/2)]
                for p in points:
                    if not navigation.covers(point_shape(p)):continue
                    for yaw in (direction,direction+math.pi/2,direction-math.pi/2):
                        first=motion_edge(start,(*p,yaw),radius)
                        if first is None:continue
                        second=motion_edge((*p,yaw),goal,radius)
                        if second is not None:return {'pieces':first['pieces']+second['pieces'],'length_m':first['length_m']+second['length_m'],'word':'one_waypoint_detour'}
                return None
            plan=select_variant_chain(layers,direct,detour)
            diagnostics[name]={'raw_variants_per_task':raw_counts,'feasible_variants_per_task':[len(v)for v in layers],
                               'pruned_variants':pruned,'direct_edges_tested':tested,'fallback_edges_tested':fallback_tested,
                               'task_ids':[t['task_id']for t in tasks]}
            if not plan['passed']:
                platforms[name]={'motion_geometry_complete':False,'failure':plan,'task_count':len(tasks)}
                (args.output_dir/(name+'_variants.json')).write_text(json.dumps(layers,indent=2)+'\n');print(name,'no complete variant chain',flush=True);continue
            selected=[layers[i][j]for i,j in enumerate(plan['variant_indices'])];pieces=[];legs=[]
            for i,v in enumerate(selected):
                if i:
                    edge=plan['edges'][i-1];pieces.extend(edge['pieces']);legs.append({'kind':'transit','scanning':False,'pieces':[asdict(p)for p in edge['pieces']],'length_m':edge['length_m'],'method':edge['word']})
                scan=line(*v['points']);pieces.append(scan);legs.append({'kind':'scan','scanning':True,'task_id':v['task_id'],'pieces':[asdict(scan)],'points':v['points'],'length_m':scan.length})
            profile=stopping_profile(pieces,cfg['cruise_mps'][name],cfg['tangential_accel_mps2'],cfg['lateral_accel_mps2'])
            platforms[name]={'motion_geometry_complete':True,'task_count':len(selected),'selected_scans':selected,
                             'legs':legs,'pieces':[asdict(p)for p in pieces],'profile':profile,'length_m':sum(p.length for p in pieces),
                             'initial_pose':pieces[0].evaluate(0)[:3],'final_pose':pieces[-1].evaluate(1)[:3],
                             'geometry_changed_task_count':sum(v['kind']not in('original','reversed')for v in selected),
                             'variant_kind_counts':{kind:sum(v['kind']==kind for v in selected)for kind in sorted({v['kind']for v in selected})}}
            print(name,'complete',platforms[name]['length_m'],profile['duration_s'],flush=True)
        # Reuse the already checked fixed support anchor poses. Whole-mission
        # cooperative timing is withheld until route and coverage gates pass.
        fixed_source=source['platforms']['usv'];knots=[fixed_source['legs'][0]['points'][0]]+[leg['points'][-1]for leg in fixed_source['legs']]
        headings=[math.atan2(knots[min(len(knots)-1,i+1)][1]-knots[max(0,i-1)][1],knots[min(len(knots)-1,i+1)][0]-knots[max(0,i-1)][0])for i in range(len(knots))]
        links=[motion_edge((*knots[i],headings[i]),(*knots[i+1],headings[i+1]),cfg['usv_minimum_radius_m'])for i in range(len(knots)-1)]
        if all(links):
            pieces=[p for link in links for p in link['pieces']]
            platforms['usv']={'motion_geometry_complete':True,'task_count':0,'pieces':[asdict(p)for p in pieces],'length_m':sum(p.length for p in pieces),
                              'profile':stopping_profile(pieces,cfg['cruise_mps']['usv'],cfg['tangential_accel_mps2'],cfg['lateral_accel_mps2'])}
        else:platforms['usv']={'motion_geometry_complete':False,'task_count':0}
        all_complete=all(v['motion_geometry_complete']for v in platforms.values());coverage={}
        if all_complete:
            scans=[footprint(leg['points'],cfg['sensor_radius_m'])for name in('uuv_1','uuv_2')for leg in platforms[name]['legs']if leg['scanning']]
            covered=g.operation('union',{'type':'GeometryCollection','geometries':scans});area=g.measure(target)['area_m2'];off_area=g.measure(covered)['area_m2']
            lower=[covered];upper=[covered]
            for name in('uuv_1','uuv_2'):
                for leg in platforms[name]['legs']:
                    if leg['scanning']or not leg['pieces']:continue
                    pp=[Piece(**p)for p in leg['pieces']];points,error=sample_enclosure(pp,cfg['maximum_arc_sagitta_m']);shape={'type':'LineString','coordinates':points}
                    lo=g.operation('buffer',shape,distance=cfg['sensor_radius_m']-error,quadrant_segments=30)
                    hi=g.operation('buffer',shape,distance=(cfg['sensor_radius_m']+error)/math.cos(math.pi/120),quadrant_segments=30)
                    lower.append(g.operation('intersection',lo,target));upper.append(g.operation('intersection',hi,target))
            onlo=g.operation('union',{'type':'GeometryCollection','geometries':lower});onhi=g.operation('union',{'type':'GeometryCollection','geometries':upper})
            coverage={'target_area_m2':area,'scan_only_covered_area_m2':off_area,'scan_only_coverage_ratio':off_area/area,
                      'turn_scan_ON_coverage_lower_bound':g.measure(onlo)['area_m2']/area,'turn_scan_ON_coverage_upper_bound':g.measure(onhi)['area_m2']/area,
                      'policy':'Primary: active scan lines only. Separate sensitivity: entire UUV transit scanning ON, ideal50m circle with chord-error inner/outer compensation; USV never scans.'}
            for filename,shape in [('scan_only_covered.geojson',covered),('scan_only_missed.geojson',g.operation('difference',target,covered)),('transit_scan_on_covered_lower.geojson',onlo)]:
                (args.output_dir/filename).write_text(json.dumps(shape)+'\n')
    result={'status':'finite_variant_route_trial_completed','platforms':platforms,'diagnostics':diagnostics,
            'all_platform_motion_geometry_complete':all_complete,'coverage':coverage,'full_mission_retiming_released':False,
            'coverage_target_sha256':cfg['A1_geometry_sha256']['searchable'],
            'method':'Finite task-variant layered graph with Dubins edges and bounded detour fallback; footprint-loss penalty20 per equivalent lost scan metre. Not globally optimal; source task IDs all retained.'}
    (args.output_dir/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    files=[args.config,args.a3_run_dir/'result.json',args.prepared_dir/'searchable.geojson',args.prepared_dir/'hazards.geojson',args.navigation_dir/'navigation.geojson']
    code=[Path(__file__),ROOT/'src/berthing_planner/dubins.py',ROOT/'src/berthing_planner/curves.py',ROOT/'src/berthing_planner/headland.py',ROOT/'src/berthing_planner/motion_feasibility.py',ROOT/'scripts/run_headland_connectors.py',ROOT/'scripts/navigation_geometry.py',ROOT/'scripts/run_motion_feasibility.py',ROOT/'scripts/chart_geometry.py']
    provenance={'input_sha256':{str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in files},'code_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in code},'result_sha256':hashlib.sha256((args.output_dir/'result.json').read_bytes()).hexdigest()}
    (args.output_dir/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    print(json.dumps({'all_platform_motion_geometry_complete':all_complete,'coverage':coverage},indent=2))


if __name__=='__main__':main()
