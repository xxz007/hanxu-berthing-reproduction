"""Plot recorded R2/R3 paths with existing workbench Matplotlib, no new search."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',type=Path,required=True)
    args=p.parse_args()
    output=ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_paper_search_plots')
    output.mkdir(exist_ok=False)
    m=json.loads((args.input_dir/'baseline_map.json').read_text())
    positions={v['name']:v['chart_EN_proxy'] for v in json.loads((args.input_dir/'positions.json').read_text())}
    x0,y0,x1,y1=m['bounds'];n=m['n']
    fig,axes=plt.subplots(2,2,figsize=(12.5,9.1))
    for ax,(goal,policy) in zip(axes.flat,(('transit_goal','local'),('transit_goal','theta_relaxation'),('stern_goal','local'),('stern_goal','theta_relaxation'))):
        r=json.loads((args.input_dir/f'baseline_{goal}_{policy}.json').read_text())
        ax.set_facecolor('#eef5fa')
        for j,row in enumerate(m['rows_bottom_to_top']):
            for i,blocked in enumerate(row):
                if blocked:
                    ax.add_patch(Rectangle((i*x1/n,j*y1/n),x1/n,y1/n,facecolor='#485f8f',edgecolor='none'))
        xs,ys=zip(*r['path'])
        ax.plot(xs,ys,color='#b54558',lw=1.8,zorder=3)
        ax.scatter(xs,ys,s=9,color='#b54558',zorder=4)
        for name,marker,label in (('start','o','start'),(goal,'*','goal')):
            xy=positions[name];ax.scatter(*xy,s=60,marker=marker,color='#087a70',edgecolor='white',zorder=5)
        title=('Transit' if goal=='transit_goal' else 'Stern position')+' | '+('ordinary A*' if policy=='local' else 'LOS-parent hypothesis')
        ax.set_title(title+f'\n{r["length_m"]:.1f} m; opened {r["opened"]}, closed {r["closed"]}',fontsize=10,loc='left',pad=9)
        ax.set(xlim=(x0,x1),ylim=(y0,y1),xlabel='Chart E proxy / m',ylabel='Chart N proxy / m',aspect='equal')
        ax.tick_params(labelsize=8)
    fig.suptitle('Original-harbor reconstruction: position-only path search',fontsize=16,y=.97)
    fig.text(.5,.035,'Independent reconstruction; no author-code equivalence claimed. Heading, smoothing and berthing constraints are not yet applied.',ha='center',fontsize=9)
    fig.subplots_adjust(left=.075,right=.98,bottom=.12,top=.9,wspace=.25,hspace=.36)
    fig.savefig(output/'paper_search_comparison.png',dpi=180,facecolor='white')
    plt.close(fig)
    print(output)


if __name__=='__main__':
    main()
