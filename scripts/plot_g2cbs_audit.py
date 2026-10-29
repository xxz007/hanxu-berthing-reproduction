"""Plot recorded R4 geometry without rerunning or changing the planner."""
import argparse
from dataclasses import fields
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
    from matplotlib.collections import PatchCollection
    from matplotlib.patches import Rectangle
    data=json.loads((args.input_dir/'records.json').read_text())
    grid=json.loads((args.input_dir/'baseline_map.json').read_text())
    x0,y0,x1,y1=grid['bounds'];n=grid['n'];dx,dy=(x1-x0)/n,(y1-y0)/n
    patches=[Rectangle((x0+i*dx,y0+j*dy),dx,dy)
             for j,row in enumerate(grid['rows_bottom_to_top']) for i,v in enumerate(row) if v]
    fig,axes=plt.subplots(2,2,figsize=(13,9),layout='constrained')
    for row,goal in enumerate(('transit_goal','stern_goal')):
        for col,kind in enumerate(('path','separate_los_shortcut')):
            ax=axes[row,col];r=data[f'baseline_{goal}_{kind}']
            # A fresh collection is required for each axes.
            ax.add_collection(PatchCollection(patches,facecolor='#586b87',edgecolor='none'))
            path=r['path'];ax.plot(*zip(*path),'o--',ms=3,lw=1,color='#9b6b43',label='Input polyline')
            for i,p in enumerate(r.get('pieces',[])):
                piece=Piece(**{f.name:p[f.name] for f in fields(Piece)})
                samples=piece.samples(4)
                blocked=i in r['blocked_piece_indices']
                ax.plot([s[0] for s in samples],[s[1] for s in samples],
                        color='#cc394c' if blocked else '#008e90',lw=2)
            for i in r['budget']['overlap_segments']:
                ax.plot(*zip(*path[i:i+2]),color='#cc394c',lw=4,label='Insufficient segment budget')
            ax.scatter(*zip(path[0],path[-1]),s=45,color='#182c47',zorder=5)
            ax.set(xlim=(x0,x1),ylim=(y0,y1),aspect='equal',xlabel='Chart E proxy (m)',ylabel='Chart N proxy (m)')
            label='Transit' if row==0 else 'Stern target position'
            treatment='Raw search nodes' if col==0 else 'Separate LOS shortcut'
            ax.set_title(f'{label} | {treatment}\n{r["status"]}',fontsize=11)
            ax.grid(alpha=.12)
    fig.suptitle('R4 audit on recovered map | Rmin = 200 m\nRed: overlap or colliding curve; teal: checked geometry; no RS or heading integration',fontsize=13)
    target=args.input_dir/'g2cbs_audit.png'
    if target.exists():
        raise FileExistsError(target)
    fig.savefig(target,dpi=150);plt.close(fig)
    (args.input_dir/'plot_provenance.json').write_text(json.dumps({
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'matplotlib':matplotlib.__version__, 'input_records_sha256':hashlib.sha256((args.input_dir/'records.json').read_bytes()).hexdigest(),
        'plot_sha256':hashlib.sha256(target.read_bytes()).hexdigest()},indent=2)+'\n')
    print(target)


if __name__=='__main__':
    main()
