"""Enclose exact ZOH quadratic trajectories and their ideal scanning footprint."""
import math
import numpy as np
from berthing_planner.experiment.navigation_geometry import NavigationGeometry


def simplify_indices(points, tolerance):
    keep={0,len(points)-1};stack=[(0,len(points)-1)]
    while stack:
        a,b=stack.pop()
        if b<=a+1:continue
        delta=points[b]-points[a];length2=float(delta@delta)
        q=points[a+1:b];u=np.clip(((q-points[a])@delta)/length2,0,1)if length2 else np.zeros(len(q))
        error=np.linalg.norm(q-(points[a]+u[:,None]*delta),axis=1);j=int(error.argmax())
        if error[j]>tolerance:
            k=a+1+j;keep.add(k);stack.extend(((a,k),(k,b)))
    return sorted(keep)


def evaluate_geometry(times, states, controls, windows, shapes, cfg):
    dt=np.diff(times);tolerance=cfg['simplification_tolerance_m'];names=('usv','uuv_1','uuv_2')
    # |p''| is the held command norm; drift is constant in each registered case.
    errors=np.max(np.linalg.norm(controls[:,:,:2],axis=2)*dt[:,None]**2/8,axis=0)+tolerance+1e-6
    paths={};clearances={};nav_ok={};scans=[];lower=[];upper=[]
    with NavigationGeometry(shapes['navigation'],shapes['hazards'])as nav:
        g=nav.geos
        for j,name in enumerate(names):
            xy=states[:,j,:2];indices=simplify_indices(xy,tolerance);points=xy[indices].tolist()
            shape={'type':'LineString','coordinates':points};paths[name]={'indices':indices,'points':points,'error_bound_m':float(errors[j])}
            envelope=g.operation('buffer',shape,distance=float(errors[j])/math.cos(math.pi/480),quadrant_segments=120)
            nav_ok[name]=nav.covers(envelope);clearances[name]=nav.distance(shape,to_hazards=True)-float(errors[j])
        for window in windows:
            j=names.index(window['platform']);start=window['global_start_s'];end=window['global_end_s']
            a=int(np.searchsorted(times,start));b=int(np.searchsorted(times,end))
            if max(abs(times[a]-start),abs(times[b]-end))>1e-7:raise ValueError('Exact scan boundaries required in trajectory grid')
            xy=states[a:b+1,j,:2];indices=simplify_indices(xy,tolerance);points=xy[indices].tolist();shape={'type':'LineString','coordinates':points}
            bound=float(np.max(np.linalg.norm(controls[a:b,j,:2],axis=1)*dt[a:b]**2/8))+tolerance+1e-6
            q=cfg['footprint_quadrant_segments'];radius=cfg['sensor_radius_m']
            lower.append(g.operation('buffer',shape,distance=radius-bound,quadrant_segments=q))
            upper.append(g.operation('buffer',shape,distance=(radius+bound)/math.cos(math.pi/(4*q)),quadrant_segments=q))
            scans.append({'task_id':window['task_id'],'platform':window['platform'],'start_index':a,'end_index':b,'indices':indices,'points':points,'error_bound_m':bound})
        merged_low=g.operation('union',{'type':'GeometryCollection','geometries':lower});merged_high=g.operation('union',{'type':'GeometryCollection','geometries':upper})
        covered=g.operation('intersection',merged_low,shapes['searchable']);missed=g.operation('difference',shapes['searchable'],merged_low)
        target_area=g.measure(shapes['searchable'])['area_m2'];lo=g.measure(covered)['area_m2']/target_area;hi=g.measure(g.operation('intersection',merged_high,shapes['searchable']))['area_m2']/target_area
    return {'target_area_m2':target_area,'coverage_lower_ratio':lo,'coverage_upper_ratio':hi,'missed_area_upper_m2':target_area*(1-lo),'navigation_enclosures_passed':nav_ok,'hazard_clearance_lower_m':clearances,'all_geometry_passed':all(nav_ok.values())and min(clearances.values())>=cfg['hazard_margin_m'],'paths':paths,'scans':scans},covered,missed
