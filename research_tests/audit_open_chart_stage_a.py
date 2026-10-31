"""Audit saved chart preparation with independent point-in-polygon and PROJ checks."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.coordinates import Mercator


def inside(geometry, points):
    kind=geometry['type']
    if kind in ('MultiPolygon','GeometryCollection'):
        members = ({'type':'Polygon','coordinates':p} for p in geometry['coordinates']) if kind=='MultiPolygon' else geometry['geometries']
        result=np.zeros(len(points),dtype=bool)
        boundary=np.zeros(len(points),dtype=bool)
        for member in members:
            member_inside,member_boundary=inside(member,points)
            result |= member_inside
            boundary |= member_boundary
        return result,boundary
    if kind!='Polygon':
        return np.zeros(len(points),dtype=bool),np.zeros(len(points),dtype=bool)
    result=np.zeros(len(points),dtype=bool)
    boundary=np.zeros(len(points),dtype=bool)
    for i,ring in enumerate(geometry['coordinates']):
        if not ring:
            continue
        xy=np.asarray(ring,dtype=float)
        tolerance=1e-7
        indices=np.flatnonzero((points[:,0]>=xy[:,0].min()-tolerance) & (points[:,0]<=xy[:,0].max()+tolerance) &
                               (points[:,1]>=xy[:,1].min()-tolerance) & (points[:,1]<=xy[:,1].max()+tolerance))
        x,y=points[indices].T
        contained=np.zeros(len(indices),dtype=bool)
        on_boundary=np.zeros(len(indices),dtype=bool)
        for a,b in zip(xy,xy[1:]):
            dx,dy=b-a
            length=np.hypot(dx,dy)
            if length:
                cross=np.abs((x-a[0])*dy-(y-a[1])*dx)/length
                along=((x-a[0])*dx+(y-a[1])*dy)/length
                on_boundary |= (cross<=tolerance) & (along>=-tolerance) & (along<=length+tolerance)
            if a[1]==b[1]:
                continue
            crossing=(a[1]>y)!=(b[1]>y)
            intercept=a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1])
            contained ^= crossing & (x<intercept)
        if i==0:
            result[indices]=contained|on_boundary
        else:
            result[indices] &= ~(contained & ~on_boundary)
        boundary[indices] |= on_boundary
    return result,boundary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--json-out',type=Path,required=True)
    parser.add_argument('--proj-bin',type=Path,default=Path('/Applications/Postgres.app/Contents/Versions/18/bin/proj'))
    args=parser.parse_args()
    p=args.prepared_dir
    cfg=json.loads((p/'config.json').read_text())
    summary=json.loads((p/'summary.json').read_text())
    shapes={name:json.loads((p/(name+'.geojson')).read_text())['features'][0]['geometry']
            for name in ('task_boundary','searchable','excluded_task')}
    xmin,ymin,xmax,ymax=summary['bbox_local_m']
    step=10.0
    xx,yy=np.meshgrid(np.arange(xmin+step/2,xmax,step),np.arange(ymin+step/2,ymax,step))
    points=np.column_stack((xx.ravel(),yy.ravel()))
    classified={name:inside(geometry,points) for name,geometry in shapes.items()}
    masks={name:values[0] for name,values in classified.items()}
    # Shared polygon boundaries are measure-zero sets and can belong to both
    # closed polygons. Audit interiors separately rather than counting such
    # grid centres as a partition error or double-covered area.
    boundary=np.logical_or.reduce([values[1] for values in classified.values()])
    audit_domain=~boundary
    source_masks={name:np.zeros(len(points),dtype=bool) for name in ('land','depth','coverage','no_coverage','entry_prohibited')}
    for layer in ('LNDARE','DEPARE','M_COVR','RESARE'):
        for f in json.loads((p/'features'/(layer+'.geojson')).read_text())['features']:
            props=f['properties']
            if layer=='LNDARE':
                key='land'
            elif layer=='DEPARE':
                if props.get('DRVAL1') is None or props['DRVAL1']<cfg['engineering_choices']['minimum_depth_lower_bound_m']:
                    continue
                key='depth'
            elif layer=='M_COVR':
                key='coverage' if props['CATCOV']==1 else 'no_coverage'
            else:
                if not set(cfg['engineering_choices']['excluded_restriction_codes']).intersection(int(v) for v in props.get('RESTRN') or []):
                    continue
                key='entry_prohibited'
            source_masks[key] |= inside(f['geometry'],points)[0]
    candidate=masks['searchable']
    violations={
        'outside_task':int(np.sum(audit_domain & candidate & ~masks['task_boundary'])),
        'outside_chart_coverage':int(np.sum(audit_domain & candidate & (~source_masks['coverage'] | source_masks['no_coverage']))),
        'outside_qualified_depth_band':int(np.sum(audit_domain & candidate & ~source_masks['depth'])),
        'on_source_land':int(np.sum(audit_domain & candidate & source_masks['land'])),
        'in_prohibited_restriction':int(np.sum(audit_domain & candidate & source_masks['entry_prohibited'])),
        'task_partition_gap':int(np.sum(audit_domain & masks['task_boundary'] & ~(candidate | masks['excluded_task']))),
        'task_partition_overlap':int(np.sum(audit_domain & candidate & masks['excluded_task']))}
    pconf=cfg['projection']
    projection=Mercator(a=pconf['a_m'],b=pconf['b_m'],lon0_deg=pconf['central_longitude_deg'],lat_ts_deg=pconf['true_scale_latitude_deg'])
    offset=projection.forward(pconf['central_longitude_deg'],pconf['origin_latitude_deg'])[1]
    bbox=cfg['bbox_lon_lat']
    lonlat=[(lon,lat) for lon in np.linspace(bbox[0],bbox[2],8) for lat in np.linspace(bbox[1],bbox[3],8)]
    expected=np.array([(projection.forward(float(lon),float(lat))[0],projection.forward(float(lon),float(lat))[1]-offset) for lon,lat in lonlat])
    command=[str(args.proj_bin),'-f','%.12f',*summary['projection_proj_string'].split()]
    forward=subprocess.run(command,input=''.join(f'{lon:.12f} {lat:.12f}\n' for lon,lat in lonlat),capture_output=True,text=True,check=True)
    actual=np.array([[float(v) for v in row.split()[:2]] for row in forward.stdout.splitlines()])
    inverse=subprocess.run([str(args.proj_bin),'-I',*command[1:]],input=''.join(f'{x:.12f} {y:.12f}\n' for x,y in actual),capture_output=True,text=True,check=True)
    reversed_points=np.array([[float(v) for v in row.split()[:2]] for row in inverse.stdout.splitlines()])
    max_forward=float(np.max(np.abs(expected-actual)))
    max_inverse=float(np.max(np.abs(reversed_points-np.array(lonlat))))
    xy=np.asarray(cfg['task_polygon_local_m'])
    task_shoelace=abs(float(np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])))/2
    result={'method':'Independent NumPy ray casting on a fixed 10 m grid; native PROJ on 64 forward and 64 inverse locations',
            'grid_centres':len(points),'boundary_grid_centres_separately_reported':int(boundary.sum()),
            'interior_grid_centres_audited':int(audit_domain.sum()),
            'boundary_policy':'1e-7 m segment-distance tolerance; shared boundaries excluded only from interior partition/violation counts, not from vector area or exported geometry',
            'task_grid_centres':int(masks['task_boundary'].sum()),
            'candidate_grid_centres':int(candidate.sum()),'violations':violations,
            'candidate_raster_area_m2':int(candidate.sum())*step*step,
            'candidate_vector_area_m2':summary['searchable_area_m2'],
            'task_shoelace_area_m2':task_shoelace,
            'projection_forward_locations':len(lonlat),'projection_inverse_locations':len(lonlat),
            'max_forward_difference_m':max_forward,'max_inverse_difference_deg':max_inverse,
            'native_projection_version':subprocess.run([str(args.proj_bin)],capture_output=True,text=True).stderr.splitlines()[:1],
            'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'limits':'Grid-centre audit is finite sampling; full vector exclusion/clearance checks are recorded separately by the producer. No trajectory checked.'}
    result['passed']=not any(violations.values()) and max_forward<1e-5 and max_inverse<1e-9 and abs(task_shoelace-summary['task_area_m2'])<1e-7
    args.json_out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
