"""Recovered-map RS integration audit, retaining canonical failures explicitly."""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import g2_smooth,collision_free,curvature_maximum
from berthing_planner.g2_bisection import bisect_corners
from berthing_planner.raster_map import RasterQuadTree
from berthing_planner.stitching import stitch_rs


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--search-dir',type=Path,required=True)
    parser.add_argument('--map-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    output=args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_paper_combination_r5_r6')
    output.mkdir(parents=True,exist_ok=False)
    cases={};bisections={};input_hashes={}
    for variant in ('tight','baseline','loose'):
        map_file=args.map_dir/(variant+'_map.json')
        tree=RasterQuadTree(json.loads(map_file.read_text()))
        input_hashes[str(map_file.resolve())]=hashlib.sha256(map_file.read_bytes()).hexdigest()
        for goal in ('transit_goal','stern_goal'):
            source=args.search_dir/f'{variant}_{goal}_theta_relaxation.json'
            raw=json.loads(source.read_text());path=raw['path']
            input_hashes[str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
            for field in ('path','separate_los_shortcut'):
                r=bisect_corners(raw[field],.005)
                if r['pieces']:
                    r['maximum_actual_curvature_per_m']=max(map(curvature_maximum,r['pieces']))
                    r['collision_free']=collision_free(r['pieces'],tree.obstacles,tree.bounds)
                r['pieces']=[asdict(p) for p in r['pieces']]
                bisections[f'{variant}_{goal}_{field}']=r
            attempts=[]
            for n in range(len(path),1,-1):
                try:
                    spine=g2_smooth(path[:n],.005)
                except ValueError as error:
                    if not any(word in str(error) for word in ('overlap','exceeds')):
                        raise
                    attempts.append({'original_point_count':n,'rejection':str(error)})
                    continue
                break
            for heading_name,yaw in (('journal_E0',0.0),('thesis_formula_axis_swap',math.pi/2)):
                for connector in (('canonical','canonical_plus_stern_axis') if goal=='stern_goal' else ('canonical',)):
                    key=f'{variant}_{goal}_{heading_name}_{connector}'
                    r=stitch_rs(spine,(*path[0],yaw),(*path[-1],yaw),.005,tree.obstacles,tree.bounds,
                                raw['length_m'],final_direction=-1 if goal=='stern_goal' else None,
                                stern_axis_radii=[200.0] if connector!='canonical' else [])
                    r.update(variant=variant,goal_name=goal,heading_interpretation=heading_name,
                             start_pose=(*path[0],yaw),goal_pose=(*path[-1],yaw),
                             terminal_connector=connector,reverse_weight=2.0,maximum_allowed_curvature_per_m=.005,
                             original_search_path=path,original_point_count=len(path),
                             prefix_original_point_count=n,prefix_attempts=attempts,
                             no_los_postprocessing_applied=True,
                             full_spine_geometry_collision_free=collision_free(spine,tree.obstacles,tree.bounds),
                             spine_pieces=[asdict(p) for p in spine])
                    r['pieces']=[asdict(p) for p in r['pieces']]
                    cases[key]=r
                    (output/(key+'.json')).write_text(json.dumps(r,indent=2)+'\n')
                    print(key,r['status'],round(r.get('length_m',0),2),flush=True)
    (output/'bisection_audit.json').write_text(json.dumps(bisections,indent=2)+'\n')
    summary={k:{p:v for p,v in r.items() if p not in ('pieces','trace','spine_pieces','original_search_path','prefix_attempts')}
             for k,r in cases.items()}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    shutil.copy2(args.map_dir/'baseline_map.json',output/'baseline_map.json')
    hashes={}
    for path in [Path(__file__),*sorted((ROOT/'src/berthing_planner').glob('*.py')),
                 ROOT/'docs/g2cbs_source_manifest.json',ROOT/'docs/g2cbs_2008_source_manifest.json']:
        target=output/'code_snapshot'/path.relative_to(ROOT)
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
        hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    (output/'provenance.json').write_text(json.dumps({
        'created_utc':datetime.now(timezone.utc).isoformat(),'python':platform.python_version(),
        'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'source_sha256':hashes,'input_sha256':input_hashes,
        'source_conditions':{'minimum_radius_m':200,'terminal_activation_fraction':2/3,'terminal_interval_m':100},
        'engineering_choices':['first-third initial RS anchors and cost selection','reverse weight 2',
                              'activation origin uses full A-star polyline length with quadrature along G2 prefix',
                              'explicit noncanonical forward-arc/reverse-axis extension for stern case'],
        'heading_evidence':'journal p6 explicit EN poses; thesis p42 figure visually supports eastward terminal tangent; alternate mathematical axis-swap branch kept',
        'not_completed':['Han short-turn merging / full Fig3.8','author code/map equivalence','table3.3 strict reproduction','R7 comparisons','R8 coordinates/replanning'],
        'validation_scope':'continuous centerline geometry against recovered blocked rectangles; no hull or dynamics',
        'R4_full_gate_passed':False,'source_pdfs_uploaded':False},indent=2)+'\n')
    print(output,flush=True)


if __name__=='__main__':
    main()
