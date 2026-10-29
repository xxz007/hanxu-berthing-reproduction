"""Render saved original-map combination results without rerunning planning."""
import argparse
import hashlib
import json
from pathlib import Path
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
    x0,y0,x1,y1=grid['bounds'];n=grid['n'];dx,dy=(x1-x0)/n,(y1-y0)/n
    fig,axes=plt.subplots(1,2,figsize=(14,5.5),layout='constrained')
    for ax,goal,connector in zip(axes,('transit_goal','stern_goal'),('canonical','canonical_plus_stern_axis')):
        filename=f'baseline_{goal}_journal_E0_{connector}.json'
        r=json.loads((args.input_dir/filename).read_text())
        patches=[Rectangle((x0+i*dx,y0+j*dy),dx,dy)
                 for j,row in enumerate(grid['rows_bottom_to_top']) for i,v in enumerate(row) if v]
        ax.add_collection(PatchCollection(patches,facecolor='#5f718a',edgecolor='none'))
        ax.plot(*zip(*r['original_search_path']),'o--',color='#999999',ms=3,lw=1,label='Original search nodes')
        if r['status']=='retained_route_passed':
            for i,p in enumerate(r['pieces']):
                piece=Piece(**p);samples=piece.samples(4)
                ax.plot([s[0] for s in samples],[s[1] for s in samples],
                    color='#c33e59' if piece.direction<0 else '#007f86',
                    linestyle='--' if piece.direction<0 else '-',lw=2)
            for point in r['cusps']:
                ax.scatter(*point,s=55,facecolors='white',edgecolors='#c33e59',linewidth=1.5,zorder=6)
        for name in ('start_pose','goal_pose'):
            p=r[name];ax.scatter(*p[:2],s=30,color='#162f51',zorder=6)
            ax.arrow(p[0],p[1],170,0,width=8,head_width=65,color='#162f51',zorder=7,length_includes_head=True)
        label='Transit | canonical RS candidates' if goal=='transit_goal' else 'Stern | explicit S+ C+ | S- extension'
        metrics=f'L = {r.get("length_m",0):.1f} m, reverse = {r.get("reverse_m",0):.1f} m; {r["status"]}'
        ax.set_title(label+'\n'+metrics,fontsize=10)
        ax.set(xlim=(x0,x1),ylim=(y0,y1),aspect='equal',xlabel='Chart E proxy (m)',ylabel='Chart N proxy (m)')
        ax.grid(alpha=.12)
    fig.suptitle('Recovered-map integration | Rmin = 200 m | journal heading E0\nTeal: forward; dashed red: reverse; circles: direction switches; arrows: endpoint body headings',fontsize=12)
    target=args.input_dir/'paper_combination.png'
    if target.exists():
        raise FileExistsError(target)
    fig.savefig(target,dpi=170);plt.close(fig)
    (args.input_dir/'plot_provenance.json').write_text(json.dumps({
        'matplotlib':matplotlib.__version__,
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'plot_sha256':hashlib.sha256(target.read_bytes()).hexdigest()},indent=2)+'\n')
    print(target)


if __name__=='__main__':
    main()
