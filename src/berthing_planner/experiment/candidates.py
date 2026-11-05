"""Finite variant-graph trial: endpoint retreats/extensions and short through-scans.

Preserve every task ID and the full original coverage target. All adaptations,
missing tasks and failed connections remain explicit. No arbitrary skipped
task can turn a partial route into a reported complete mission.
"""
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
from berthing_planner.curves import Piece,line
from berthing_planner.dubins import candidates
from berthing_planner.headland import reachable_boxes,scan_variants,select_variant_chain
from berthing_planner.motion_feasibility import stopping_profile
from .navigation_geometry import NavigationGeometry
from .curve_geometry import sample_enclosure


def rings(shape):
    if shape['type']=='Polygon':yield from shape['coordinates']
    elif shape['type']=='MultiPolygon':
        for p in shape['coordinates']:yield from p
    elif shape['type']=='GeometryCollection':
        for g in shape['geometries']:yield from rings(g)


def run(*, allocation_dir, prepared_dir, navigation_dir, config, output_dir):
    cfg=json.loads(config.read_text());source=json.loads((allocation_dir/'result.json').read_text())
    scientific={k:v for k,v in source.items()if k!='total_computation_seconds'}
    if hashlib.sha256(json.dumps(scientific,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()!=cfg['source_a3_scientific_result_sha256']:
        raise ValueError('A3 source scientific result changed')
    shapes={}
    for name in ('searchable','hazards'):
        p=prepared_dir/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg['A1_geometry_sha256'][name]:raise ValueError('A1 scientific geometry changed')
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    navpath=navigation_dir/'navigation.geojson';navsummary=json.loads((navigation_dir/'summary.json').read_text())
    if hashlib.sha256(navpath.read_bytes()).hexdigest()!=navsummary['navigation_sha256']:raise ValueError('Navigation changed')
    if navsummary['navigation_sha256']!=cfg['expected_navigation_sha256']:raise ValueError('Navigation does not match fixed A5 geometry')
    nav=json.loads(navpath.read_text())['features'][0]['geometry'];target=shapes['searchable'];hazards=shapes['hazards']
    output_dir.mkdir(parents=True,exist_ok=False);(output_dir/'config.json').write_bytes(config.read_bytes())
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
            (output_dir/(name+'_variants.json')).write_text(json.dumps(layers,indent=2)+'\n')
            diagnostics[name]={'layer_count':len(layers),'variant_counts':[len(v)for v in layers],'pruned_count':pruned}
    (output_dir/'summary.json').write_text(json.dumps({'purpose':'Navigable scan candidate layers for joint curve planning','platforms':diagnostics},indent=2)+'\n')
