"""Plot saved R7 paths and all timing observations; no planner reruns."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir',type=Path,required=True)
    args=parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from matplotlib.collections import PatchCollection
    grid=json.loads((args.input_dir/'baseline_map.json').read_text())
    summary=json.loads((args.input_dir/'summary.json').read_text())
    x0,y0,x1,y1=grid['bounds'];n=grid['n'];dx,dy=(x1-x0)/n,(y1-y0)/n
    methods=['multi_astar_rs','jps','smooth_rrt_30','smooth_rrt_60']
    labels=['Multi-scale A* + RS (independent)','Online JPS | 10 m, no corner cutting',
            'Smooth-RRT 30 deg | guarded interpretation','Smooth-RRT 60 deg | guarded interpretation']
    colors=['#00878c','#336da5','#be7724','#ad4e83']
    fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
    input_hashes={}
    for ax,method,label,color in zip(axes.flat,methods,labels,colors):
        paths=sorted(args.input_dir.glob(f'baseline_{method}_*.json'))
        records=[]
        for p in paths:
            input_hashes[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
            records.append(json.loads(p.read_text()))
        successes=[r for r in records if r['path_found']]
        patches=[Rectangle((x0+i*dx,y0+j*dy),dx,dy)
            for j,row in enumerate(grid['rows_bottom_to_top']) for i,v in enumerate(row) if v]
        ax.add_collection(PatchCollection(patches,facecolor='#68788d',edgecolor='none'))
        if successes:
            median=statistics.median(r['length_m'] for r in successes)
            record=min(successes,key=lambda r:abs(r['length_m']-median))
            r=record['result']
            if method=='multi_astar_rs':
                for p in r['pieces']:
                    samples=Piece(**p).samples(4)
                    ax.plot([s[0] for s in samples],[s[1] for s in samples],color=color,lw=2)
            else:
                path=r['physical_path'] if method=='jps' else r['path']
                if method.startswith('smooth'):
                    ax.plot(*zip(*r['raw_path']),color='#b8b8b8',lw=.8,alpha=.8)
                ax.plot(*zip(*path),color=color,lw=2)
                ax.scatter(*zip(*path),color=color,s=8)
            ax.scatter([4138,2548],[417,3777],s=35,color='#182d48',zorder=6)
        s=summary['baseline_'+method]
        length=s['length_m_successes']['median'] if s['length_m_successes'] else 0
        seconds=s['planning_seconds_all_attempts']['median']
        ax.set_title(f'{label}\nmedian L={length:.1f} m | median T={seconds:.3f}s | found {s["successes"]}/{s["attempts"]}',fontsize=10)
        ax.set(xlim=(x0,x1),ylim=(y0,y1),aspect='equal',xlabel='Chart E proxy (m)',ylabel='Chart N proxy (m)')
        ax.grid(alpha=.12)
    fig.suptitle('Recovered-map method comparison | all exact physical endpoints retained\nRRT panels show successful paths nearest the median length; grey = unpruned goal chain',fontsize=13)
    target=args.input_dir/'method_paths.png'
    if target.exists():raise FileExistsError(target)
    fig.savefig(target,dpi=160);plt.close(fig)
    methods=['grid_astar','jps','multi_astar_rs','smooth_rrt_30','smooth_rrt_60']
    fig,ax=plt.subplots(figsize=(10,5),layout='constrained')
    for i,method in enumerate(methods):
        records=[json.loads(p.read_text()) for p in sorted(args.input_dir.glob(f'baseline_{method}_*.json'))]
        values=[r['planning_seconds'] for r in records]
        xs=[i+(j-(len(values)-1)/2)*.025 for j in range(len(values))]
        ax.scatter(xs,values,s=25,alpha=.75)
        ax.plot([i-.15,i+.15],[statistics.median(values)]*2,color='#172e4c',lw=2.5)
    ax.set_xticks(range(len(methods)),['Grid A*\nsearch only','JPS\nsearch only','Multi A* + RS\nfull planning','RRT 30 deg\ngrowth + greedy','RRT 60 deg\ngrowth + greedy'])
    ax.set_yscale('log');ax.set_ylabel('Wall-clock seconds (log scale)')
    ax.grid(axis='y',alpha=.2)
    ax.set_title('All baseline timing observations, including failures if present\nDifferent output constraints/scopes: only Grid A* versus JPS is the same-graph search comparison')
    timing=args.input_dir/'method_timing.png'
    if timing.exists():raise FileExistsError(timing)
    fig.savefig(timing,dpi=160);plt.close(fig)
    (args.input_dir/'plot_provenance.json').write_text(json.dumps({
        'matplotlib':matplotlib.__version__,'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'input_sha256':input_hashes,'output_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (target,timing)}},indent=2)+'\n')
    print(target);print(timing)


if __name__=='__main__':main()
