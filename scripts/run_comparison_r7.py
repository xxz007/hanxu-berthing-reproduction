"""Source-bounded R7 comparison; planning scopes and constraint gaps explicit."""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import shutil
import statistics
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import g2_smooth,wrap
from berthing_planner.jps import SquareGrid,search_grid
from berthing_planner.raster_map import RasterQuadTree
from berthing_planner.search import astar
from berthing_planner.smooth_rrt import smooth_rrt,greedy_path
from berthing_planner.stitching import stitch_rs


def describe(values):
    if not values:return None
    ordered=sorted(values)
    def percentile(p):
        index=p*(len(ordered)-1);lo=math.floor(index);hi=math.ceil(index)
        return ordered[lo]+(index-lo)*(ordered[hi]-ordered[lo])
    return {'n':len(values),'median':statistics.median(values),
            'q25':percentile(.25),'q75':percentile(.75),'minimum':ordered[0],'maximum':ordered[-1]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/comparison_r7.json')
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args();config=json.loads(args.config.read_text())
    if config['schema_version']!=1 or config['warmup_runs_per_method']!=1:
        raise ValueError('This protocol expects schema1 and one warmup per method')
    checks=json.loads((args.map_dir/'checks.json').read_text())
    if not checks['reconstructed_map_gate_passed']:raise ValueError('R1 gate has not passed')
    positions={p['name']:tuple(p['chart_EN_proxy']) for p in json.loads((args.map_dir/'positions.json').read_text())}
    start,goal=positions['start'],positions['transit_goal']
    output=args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_comparison_r7')
    output.mkdir(parents=True,exist_ok=False)
    (output/'config.json').write_bytes(args.config.read_bytes())
    records=[];warmups=[];preprocessing={};input_hashes={}
    rng=random.Random(config['shuffle_seed'])
    for variant in ('baseline','tight','loose'):
        source=args.map_dir/(variant+'_map.json');input_hashes[str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
        begin=time.perf_counter();tree=RasterQuadTree(json.loads(source.read_text()));tree_seconds=time.perf_counter()-begin
        begin=time.perf_counter();grid=SquareGrid.from_rectangles(tree.bounds,tree.obstacles,config['uniform_cell_size_m']);grid_seconds=time.perf_counter()-begin
        s,g=grid.locate(start),grid.locate(goal)
        preprocessing[variant]={'quadtree_seconds':tree_seconds,'uniform_grid_seconds':grid_seconds,
            'grid_shape':[grid.nx,grid.ny],'grid_size_m':grid.cell_size,
            'source_bounds':tree.bounds,'partial_outer_cells_blocked':True,
            'grid_start_cell':s,'grid_goal_cell':g,
            'endpoint_attachment_m':math.dist(start,grid.center(s))+math.dist(goal,grid.center(g))}
        if variant=='baseline':shutil.copy2(source,output/'baseline_map.json')

        def run(method,seed):
            if method in ('jps','grid_astar'):
                return search_grid(grid,s,g,'jps' if method=='jps' else 'astar')
            if method=='multi_astar_rs':
                search=astar(tree,start,goal,parent_policy='theta_relaxation')
                attempts=[]
                for n in range(len(search['path']),1,-1):
                    try:spine=g2_smooth(search['path'][:n],config['maximum_curvature_per_m'])
                    except ValueError as error:
                        if not any(word in str(error) for word in ('overlap','exceeds')):raise
                        attempts.append({'points':n,'rejection':str(error)});continue
                    break
                r=stitch_rs(spine,(*start,config['initial_yaw_rad']),(*goal,config['goal_yaw_rad']),
                    config['maximum_curvature_per_m'],tree.obstacles,tree.bounds,search['length_m'],config['reverse_weight'])
                r.update(search_opened=search['opened'],search_closed=search['closed'],
                         prefix_points=n,prefix_attempts=attempts)
                return r
            theta=float(method.split('_')[-1]);c=config['rrt']
            return smooth_rrt(tree,start,goal,step=c['step_m'],maximum_turn=math.radians(theta),
                goal_bias=c['goal_bias'],max_iterations=c['max_iterations'],seed=seed,
                initial_yaw=config['initial_yaw_rad'],greedy_policy=c['greedy_policy'])

        methods=['grid_astar','jps','multi_astar_rs']+[f'smooth_rrt_{v:g}' for v in config['rrt']['maximum_turn_degrees']]
        for method in methods:
            begin=time.perf_counter();r=run(method,-1)
            warmups.append({'variant':variant,'method':method,'seed':-1,'status':r['status'],'seconds':time.perf_counter()-begin})
            print('warmup',variant,method,r['status'],flush=True)
        seeds=config['rrt']['baseline_seeds' if variant=='baseline' else 'sensitivity_seeds']
        jobs=[(m,i) for m in methods[:3] for i in range(config['deterministic_repeats'])]+[(m,seed) for m in methods[3:] for seed in seeds]
        rng.shuffle(jobs)
        for method,seed in jobs:
            begin=time.perf_counter();r=run(method,seed);seconds=time.perf_counter()-begin
            record={'variant':variant,'method':method,'replicate_or_seed':seed,'planning_seconds':seconds,'result':r}
            success=r['status'] in ('path_found','path_found_greedy_failed','retained_route_passed')
            record['path_found']=success
            if method=='multi_astar_rs':
                record['timing_scope']='search + G2 prefix + initial/terminal RS + internal final checks'
                record['constraints_met']=success
                if success:record['length_m']=r['length_m']
                r['pieces']=[asdict(p) for p in r['pieces']]
            else:
                record['timing_scope']='grid search only' if method in methods[:2] else 'tree growth + angle-guarded greedy polyline'
                if success:
                    if method in methods[:2]:
                        path=[start]+[grid.center(p) for p in r['cell_path']]+[goal]
                        path=[p for i,p in enumerate(path) if i==0 or p!=path[i-1]]
                        r['physical_path']=path
                    else:
                        path=r['path']
                        r['incoming_only_control']=greedy_path(tree,r['raw_path'],config['initial_yaw_rad'],r['maximum_turn_rad'],'incoming_only')
                        if r['maximum_actual_turn_rad']>r['maximum_turn_rad']+1e-9:
                            raise AssertionError('Angle factor was violated')
                    if not all(tree.line_of_sight(a,b) for a,b in zip(path,path[1:])):
                        raise AssertionError('Continuous closed-rectangle validation failed')
                    if math.dist(path[0],start)>1e-8 or math.dist(path[-1],goal)>1e-8:
                        raise AssertionError('Exact endpoints were moved')
                    angles=[math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(path,path[1:])]
                    turns=sum(abs(wrap(b-a))>1e-7 for a,b in zip(angles,angles[1:]))
                    record['polyline_turns']=turns
                    record['finite_curvature_bound_met']=turns==0
                    record['endpoint_heading_errors_rad']=[abs(wrap(angles[0]-config['initial_yaw_rad'])),abs(wrap(angles[-1]-config['goal_yaw_rad']))]
                    record['constraints_met']=turns==0 and max(record['endpoint_heading_errors_rad'])<1e-7
                    record['length_m']=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
                else:record['constraints_met']=False
            key=f'{variant}_{method}_{seed}'
            (output/(key+'.json')).write_text(json.dumps(record,indent=2)+'\n')
            records.append(record)
            print(key,r['status'],f'{seconds:.3f}s',round(record.get('length_m',0),1),flush=True)
    summaries={}
    for variant in ('baseline','tight','loose'):
        for method in methods:
            subset=[r for r in records if r['variant']==variant and r['method']==method]
            successes=[r for r in subset if r['path_found']]
            count_field='opened_unique' if method in methods[:2] else 'search_opened' if method=='multi_astar_rs' else 'tree_nodes'
            summaries[f'{variant}_{method}']={'attempts':len(subset),'successes':len(successes),
                'constraints_met':sum(r['constraints_met'] for r in subset),
                'planning_seconds_all_attempts':describe([r['planning_seconds'] for r in subset]),
                'planning_seconds_successes':describe([r['planning_seconds'] for r in successes]),
                'length_m_successes':describe([r['length_m'] for r in successes]),
                'count_definition':count_field,'node_count_all_attempts':describe([r['result'][count_field] for r in subset]),
                'incoming_only_greedy_stalls':sum(r['result'].get('incoming_only_control',{}).get('status')=='stalled_after_shortcut' for r in successes)}
    (output/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
    (output/'preprocessing.json').write_text(json.dumps(preprocessing,indent=2)+'\n')
    (output/'warmups.json').write_text(json.dumps(warmups,indent=2)+'\n')
    hashes={}
    for path in [Path(__file__),args.config,ROOT/'docs/r7_sources.json',*sorted((ROOT/'src/berthing_planner').glob('*.py'))]:
        target=output/'code_snapshot'/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
        hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    (output/'provenance.json').write_text(json.dumps({'created_utc':datetime.now(timezone.utc).isoformat(),
        'python':platform.python_version(),'platform':platform.platform(),
        'git_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'source_sha256':hashes,'input_sha256':input_hashes,'source_pdfs_uploaded':False,
        'comparison':'independent reference-backed implementations; Han comparator equivalence unconfirmed',
        'strict_table3_3_reproduction':False,'R7_strict_gate_passed':False},indent=2)+'\n')
    print(output,flush=True)


if __name__=='__main__':main()
