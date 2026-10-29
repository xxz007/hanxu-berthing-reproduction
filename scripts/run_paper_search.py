"""R2/R3 position-only search on R1 reconstructed maps, with no synthetic map."""
import argparse
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
from berthing_planner.raster_map import RasterQuadTree
from berthing_planner.search import astar,shortcut


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path)
    args=parser.parse_args()
    checks=json.loads((args.map_dir/'checks.json').read_text())
    if not checks['reconstructed_map_gate_passed']:
        raise ValueError('R1 reconstructed-map gate has not passed')
    positions={p['name']:p['chart_EN_proxy'] for p in json.loads((args.map_dir/'positions.json').read_text())}
    output=args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_paper_search_r2_r3')
    if output.exists():
        raise FileExistsError('Preserving prior search evidence: '+str(output))
    results={}
    for variant in ('tight','baseline','loose'):
        map_path=args.map_dir/(variant+'_map.json')
        tree=RasterQuadTree(json.loads(map_path.read_text()))
        for goal in ('transit_goal','stern_goal'):
            for policy in ('local','theta_relaxation','paper_parent_only'):
                key=f'{variant}_{goal}_{policy}'
                # A finite candidate failure is recorded, never replaced by another policy.
                # The ValueError here is the documented no-path result from this algorithm.
                try:
                    r=astar(tree,positions['start'],positions[goal],parent_policy=policy,record_trace=True)
                except ValueError as error:
                    if str(error)!='No path on this planning graph':
                        raise
                    results[key]={'status':'no_path_under_this_policy','policy':policy,
                                  'interpretation':'parent means parent(current); no current-node fallback'}
                    continue
                for a,b in zip(r['path'],r['path'][1:]):
                    if not tree.line_of_sight(a,b):
                        raise AssertionError('Search returned a blocked segment')
                if not math.isclose(sum(math.dist(a,b) for a,b in zip(r['path'],r['path'][1:])),r['length_m'],abs_tol=1e-7):
                    raise AssertionError('Path cost does not match its coordinates')
                r['status']='path_found_and_segment_checked'
                if policy=='local':
                    dijkstra=astar(tree,positions['start'],positions[goal],heuristic_weight=0)
                    r['dijkstra_length_m']=dijkstra['length_m']
                    r['agrees_with_dijkstra_same_graph']=math.isclose(r['length_m'],dijkstra['length_m'],abs_tol=1e-7)
                    if not r['agrees_with_dijkstra_same_graph']:
                        raise AssertionError('Ordinary A* disagrees with Dijkstra on the same graph')
                else:
                    r['separate_los_shortcut']=shortcut(tree,r['path'])
                r['input_map_sha256']=hashlib.sha256(map_path.read_bytes()).hexdigest()
                results[key]=r
    output.mkdir(parents=True,exist_ok=False)
    for key,r in results.items():
        (output/(key+'.json')).write_text(json.dumps(r,indent=2)+'\n')
    summary={k:{p:v for p,v in r.items() if p not in ('path','trace','closed_points','separate_los_shortcut')} for k,r in results.items()}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    baseline=RasterQuadTree(json.loads((args.map_dir/'baseline_map.json').read_text()))
    x0,y0,x1,y1=baseline.bounds
    scale=min(670/(x1-x0),410/(y1-y0))
    svg=['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1500 1060"><rect width="1500" height="1060" fill="#f4f7f9"/><g font-family="system-ui,sans-serif" fill="#233443"><text x="35" y="40" font-size="25">Recovered original harbor: ordinary A* and LOS-parent hypothesis</text><text x="35" y="69" font-size="16">Position-only planning | Reconstructed centerlines | No endpoint shifts | R3 author-equivalence remains unverified</text>']
    for index,(goal,policy) in enumerate((('transit_goal','local'),('transit_goal','theta_relaxation'),('stern_goal','local'),('stern_goal','theta_relaxation'))):
        ox,oy=55+(index%2)*745,125+(index//2)*470
        def xy(p):return ox+(p[0]-x0)*scale,oy+(y1-p[1])*scale
        name='Transit' if goal=='transit_goal' else 'Stern position'
        policy_name='ordinary A*' if policy=='local' else 'Theta-style relaxation (hypothesis)'
        r=results[f'baseline_{goal}_{policy}']
        svg.append(f'<text x="{ox}" y="{oy-20}" font-size="20">{name}: {policy_name}</text>')
        svg.append(f'<rect x="{ox}" y="{oy}" width="{(x1-x0)*scale}" height="{(y1-y0)*scale}" fill="#eaf3f8"/>')
        for cell in baseline.leaves.values():
            a,b,c,d=cell.bounds;u,v=xy((a,d))
            svg.append(f'<rect x="{u:.2f}" y="{v:.2f}" width="{(c-a)*scale:.2f}" height="{(d-b)*scale:.2f}" fill="{"#455d8e" if cell.blocked else "none"}" stroke="#b0c0cb" stroke-width=".25"/>')
        points=' '.join(f'{xy(p)[0]:.2f},{xy(p)[1]:.2f}' for p in r['path'])
        svg.append(f'<polyline points="{points}" fill="none" stroke="#b54558" stroke-width="2.5"/>')
        for p in (positions['start'],positions[goal]):
            u,v=xy(p);svg.append(f'<circle cx="{u}" cy="{v}" r="4" fill="#187d77" stroke="white"/>')
        svg.append(f'<text x="{ox}" y="{oy+435}" font-size="15">Length {r["length_m"]:.1f} m | opened {r["opened"]} | closed {r["closed"]}</text>')
    (output/'search_comparison.svg').write_text(''.join(svg)+'</g></svg>')
    (output/'positions.json').write_bytes((args.map_dir/'positions.json').read_bytes())
    (output/'baseline_map.json').write_bytes((args.map_dir/'baseline_map.json').read_bytes())
    source_hashes={}
    for path in [Path(__file__),ROOT/'src/berthing_planner/search.py',ROOT/'src/berthing_planner/raster_map.py',ROOT/'src/berthing_planner/quadtree.py']:
        target=output/'code_snapshot'/path.relative_to(ROOT)
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
        source_hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    git=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True)
    dirty=subprocess.run(['git','status','--porcelain'],cwd=ROOT,capture_output=True,text=True)
    (output/'environment.json').write_text(json.dumps({'created_utc':datetime.now(timezone.utc).isoformat(),
        'python':platform.python_version(),'platform':platform.platform(),'third_party_runtime_dependencies':[],
        'git_commit':git.stdout.strip() if git.returncode==0 else None,'git_dirty':bool(dirty.stdout),
        'source_sha256':source_hashes,'input_map_sha256':{v:hashlib.sha256((args.map_dir/(v+'_map.json')).read_bytes()).hexdigest() for v in ('tight','baseline','loose')},
        'positions_sha256':hashlib.sha256((args.map_dir/'positions.json').read_bytes()).hexdigest()},indent=2)+'\n')
    (output/'scope.json').write_text(json.dumps({'R2':'ordinary A-star checked against Dijkstra on the same recovered graph',
        'R3':'two explicit interpretations; no claim of author-code equivalence',
        'not_included':['heading constraints','G2CBS','RS','dynamic tracking','table3.3 numerical reproduction'],
        'neighbor':'free leaves sharing positive-length edge','los':'closed blocked-leaf rectangles'},indent=2)+'\n')
    print(output)
    for key,r in summary.items():
        print(key,r['status'],round(r.get('length_m',0),1))


if __name__=='__main__':
    main()
