"""Build README figures from saved results only, without source-map imagery.

Use the existing research plotting environment. Planner/runtime dependencies
remain unchanged. This script never reruns planning or overwrites assets.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--combination-dir',type=Path,required=True)
    parser.add_argument('--comparison-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'docs/assets')
    args=parser.parse_args()
    files=[args.combination_dir/'baseline_transit_goal_journal_E0_canonical.json',
           args.combination_dir/'baseline_stern_goal_journal_E0_canonical_plus_stern_axis.json',
           args.comparison_dir/'summary.json']
    transit,stern,summary=[json.loads(p.read_text()) for p in files]
    if any(r['status']!='retained_route_passed' for r in (transit,stern)):
        raise ValueError('Only validated saved routes may be presented as results')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    targets=[args.output_dir/name for name in ('berthing_results.png','method_lengths.png','manifest.json')]
    if any(p.exists() for p in targets):
        raise FileExistsError('Preserving existing README figures; use a new output directory')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','axes.unicode_minus':False,
        'font.size':10,'axes.titleweight':'medium','figure.facecolor':'white','savefig.facecolor':'white'})
    forward,reverse,ink='#087f8c','#cd4c68','#19334b'
    fig,axes=plt.subplots(1,2,figsize=(12.8,7.2))
    fig.subplots_adjust(top=.77,bottom=.13,wspace=.16)
    for ax,r,title in zip(axes,(transit,stern),('通航到达','船艉倒车抵泊')):
        samples_all=[]
        for data in r['pieces']:
            piece=Piece(**data);samples=piece.samples(5)
            samples_all.extend(samples)
            color=reverse if piece.direction<0 else forward
            ax.plot([s[0] for s in samples],[s[1] for s in samples],
                color=color,lw=2.7,linestyle='--' if piece.direction<0 else '-',solid_capstyle='round')
        for sign,color in ((1,forward),(-1,reverse)):
            samples=[s for s in samples_all if s[-1]==sign]
            if len(samples)>15:
                i=len(samples)//2
                ax.annotate('',xy=samples[min(i+10,len(samples)-1)][:2],xytext=samples[max(0,i-10)][:2],
                    arrowprops={'arrowstyle':'-|>','color':color,'lw':2,'mutation_scale':13})
        first,last=r['start_pose'],r['goal_pose']
        ax.scatter(*first[:2],s=45,color=ink,zorder=6)
        ax.scatter(*last[:2],s=75,marker='*',color=ink,zorder=6)
        ax.annotate('起点',first[:2],xytext=(8,-15),textcoords='offset points',color=ink)
        ax.annotate('终点',last[:2],xytext=(-35,11),textcoords='offset points',color=ink)
        for point in r['cusps']:
            ax.scatter(*point,s=75,facecolors='white',edgecolors=reverse,lw=2,zorder=7)
            ax.annotate('换向点',point,xytext=(7,9),textcoords='offset points',color=reverse)
        ax.set_title(f'{title}\n路径 {r["length_m"]:.1f} m · 倒车 {r["reverse_m"]:.1f} m',fontsize=12,pad=13,color=ink)
        ax.set(xlim=(1500,5000),ylim=(0,4250),aspect='equal',xlabel='图上 E 坐标（m）',ylabel='图上 N 坐标（m）')
        ax.grid(alpha=.16);ax.set_axisbelow(True)
        ax.spines[['top','right']].set_visible(False)
        ax.spines[['left','bottom']].set_color('#bfccd5')
    fig.suptitle('恢复港口图实验 · 两种靠泊轨迹',fontsize=19,y=.98,color=ink)
    fig.legend(handles=[Line2D([],[],color=forward,lw=2.7,label='前进'),
        Line2D([],[],color=reverse,lw=2.7,linestyle='--',label='倒车'),
        Line2D([],[],color=reverse,marker='o',markerfacecolor='white',lw=0,label='方向切换')],
        loc='upper center',bbox_to_anchor=(.5,.91),ncol=3,frameon=False)
    fig.text(.5,.035,'仅展示本项目生成的轨迹；港口底图保留本地。两例实际曲率上界均为 0.005 /m。',ha='center',color='#586b7d',fontsize=10)
    fig.savefig(targets[0],dpi=170);plt.close(fig)

    methods=['multi_astar_rs','jps','smooth_rrt_30','smooth_rrt_60']
    labels=['多尺度 A* + RS','JPS · 10 m 网格','Smooth-RRT · 30°','Smooth-RRT · 60°']
    colors=[forward,'#5b83a2','#bc8a4b','#b16c8e']
    fig,ax=plt.subplots(figsize=(12,4.6))
    fig.subplots_adjust(left=.20,right=.98,bottom=.19,top=.74)
    for i,(method,color) in enumerate(zip(methods,colors)):
        r=summary['baseline_'+method];v=r['length_m_successes'];median=v['median']/1000
        ax.barh(i,median,color=color,height=.56,alpha=.9)
        ax.errorbar(median,i,xerr=[[median-v['q25']/1000],[v['q75']/1000-median]],fmt='none',
                    color=ink,capsize=4,lw=1.3)
        ax.text(.12,i,f'{v["median"]:.1f} m',va='center',color='white',weight='medium',fontsize=11)
        note='曲率与起终航向：通过' if method=='multi_astar_rs' else '折线输出：曲率条件未满足'
        ax.text(7.1,i,note,va='center',fontsize=9,color=ink if method=='multi_astar_rs' else '#7d5960')
    ax.set_yticks(range(4),labels);ax.invert_yaxis();ax.set_xlim(0,9.45)
    ax.set_xlabel('路径长度（km）');ax.grid(axis='x',alpha=.15);ax.set_axisbelow(True)
    ax.spines[['top','right','left']].set_visible(False);ax.spines['bottom'].set_color('#bfccd5')
    ax.set_xticks(range(0,8))
    fig.suptitle('独立方法对照 · 路径长度与约束结果',fontsize=17,y=.98,color=ink)
    fig.text(.5,.86,'同一恢复地图；RRT 每组 10 个种子，线段为四分位区间；确定性方法重复 5 次。',ha='center',color='#586b7d',fontsize=10)
    fig.text(.5,.025,'长度更短不等于满足靠泊约束。作者参数与表 3.3 的严格对齐仍在进行。',ha='center',color='#586b7d',fontsize=10)
    fig.savefig(targets[1],dpi=170);plt.close(fig)
    manifest={'generated_utc':datetime.now(timezone.utc).isoformat(),
        'kind':'README presentation of existing validated results; no experiment rerun',
        'contains_source_map_background':False,'contains_paper_page_or_private_notes':False,
        'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'input_sha256':{str(p.resolve().relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'output_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in targets[:2]},
        'matplotlib':matplotlib.__version__,
        'route_metrics':{name:{k:r[k] for k in ('length_m','reverse_m','maximum_curvature_per_m','cusps')} for name,r in [('transit',transit),('stern',stern)]}}
    targets[2].write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    for p in targets:print(p)


if __name__=='__main__':main()
