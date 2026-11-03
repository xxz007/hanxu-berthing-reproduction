"""A8 audited route map and categorical support-range sensitivity comparison."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece
from chart_geometry import Geos
from run_motion_feasibility import sample_enclosure


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','a7-run-dir','audit-json','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path);args=parser.parse_args();read=lambda p:json.loads(p.read_text());x=read(args.run_dir/'result.json');source=read(args.a7_run_dir/'result.json');cfg=read(args.run_dir/'config.json');audit=read(args.audit_json)
    digest=hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()
    if not audit['passed']or audit['source_result_sha256']!=digest:raise ValueError('Matching independent A8 audit required')
    canon=lambda v:hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    if canon({k:v for k,v in source.items()if k!='total_computation_seconds'})!=cfg['source_a7_scientific_sha256']:raise ValueError('A7 source science changed')
    shapes={}
    for name,key,directory in [('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir),('navigation','navigation_sha256',args.navigation_dir)]:
        path=directory/(name+'.geojson')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Plot chart changed')
        shapes[name]=read(path)['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False);window={'type':'Polygon','coordinates':[[[0,0],[5120,0],[5120,5120],[0,5120],[0,0]]]}
    with Geos()as g:
        backgrounds={n:g.operation('intersection',s,window)for n,s in shapes.items()}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.path import Path as MplPath
    from matplotlib.patches import PathPatch,Patch
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Noto Sans CJK SC','Arial','DejaVu Sans'],'font.size':10,'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'none','figure.facecolor':'white','savefig.facecolor':'white'})
    ink='#19334b';purple='#7854a2';teal='#078b8b';orange='#b97825';gray='#87929e'
    def draw(ax,shape,face):
        kind=shape['type']
        if kind=='GeometryCollection':
            for q in shape['geometries']:draw(ax,q,face)
        elif kind=='MultiPolygon':
            for q in shape['coordinates']:draw(ax,{'type':'Polygon','coordinates':q},face)
        elif kind=='Polygon':
            vertices=[];codes=[]
            for i,ring in enumerate(shape['coordinates']):
                xy=np.asarray(ring)
                if len(xy)<4:continue
                signed=np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])
                if(signed>0)!=(i==0):xy=xy[::-1]
                vertices.extend(xy);codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
            if vertices:ax.add_patch(PathPatch(MplPath(vertices,codes),facecolor=face,edgecolor='none'))
        elif kind=='MultiLineString':
            for q in shape['coordinates']:draw(ax,{'type':'LineString','coordinates':q},face)
        elif kind=='LineString':
            xy=np.asarray(shape['coordinates'])
            if len(xy):ax.plot(xy[:,0],xy[:,1],color=face,lw=.6)
        elif kind in('Point','MultiPoint'):
            xy=np.asarray([shape['coordinates']]if kind=='Point'else shape['coordinates'])
            if len(xy):ax.scatter(xy[:,0],xy[:,1],s=4,color=face)
    fig,(left,right)=plt.subplots(1,2,figsize=(13,8.6));fig.subplots_adjust(left=.065,right=.98,bottom=.24,top=.81,wspace=.17)
    left.set_facecolor('#f0f1f2');draw(left,backgrounds['navigation'],'#d9efed');draw(left,backgrounds['searchable'],'#dce8f3');draw(left,backgrounds['hazards'],'#8b929a')
    for n,color in [('uuv_1',teal),('uuv_2',orange)]:
        xy=np.asarray(sample_enclosure([Piece(**p)for p in x['uuv_platforms'][n]['pieces']],.02)[0]);left.plot(xy[:,0],xy[:,1],color=color,lw=.55,alpha=.7)
    old=np.asarray(sample_enclosure([Piece(**p)for p in source['platforms']['usv']['pieces']],.02)[0]);left.plot(old[:,0],old[:,1],color=gray,ls='--',lw=1.5)
    for leg in x['mobile_usv']['legs']:
        xy=np.asarray(sample_enclosure([Piece(**p)for p in leg['pieces']],.02)[0]);left.plot(xy[:,0],xy[:,1],color=purple,lw=1.2);left.scatter(*xy[-1],s=16,color=purple,zorder=6)
    left.scatter(*x['mobile_usv']['initial_pose'][:2],s=60,marker='*',color=purple,zorder=7)
    left.set(xlim=(0,5120),ylim=(0,5120),aspect='equal',xlabel='东向 / m',ylabel='北向 / m');left.set_title('相同 UUV 任务，USV 分 11 次转场支援',loc='left',fontsize=12,pad=15,color=ink)
    categories={'certified_pass':('o',teal),'witnessed_strategy_violation':('X',orange),'initial_condition_impossible':('s',gray),'unresolved_bounds':('D','#b74d49')}
    rows=x['range_sweep']
    for i,row in enumerate(rows):
        for j,strategy in enumerate(('a7_fixed_station','scheduled_mobile')):
            marker,color=categories[row[strategy]];right.scatter(j,i,marker=marker,s=95,color=color,zorder=5)
    right.set(xlim=(-.7,1.7),ylim=(len(rows)-.5,-.5),xticks=[0,1],xticklabels=['A7 原驻守','A8 分段移动'],yticks=np.arange(len(rows)),yticklabels=[str(r['support_range_m'])for r in rows],ylabel='支援范围 / m（各行独立条件）');right.set_box_aspect(1);right.tick_params(axis='x',length=0,pad=10)
    right.set_title('固定两套方案，逐项收紧支援范围',loc='left',fontsize=12,pad=15,color=ink)
    right.grid(axis='y',color='#e6edf0',lw=.7);right.spines[['top','right']].set_visible(False)
    for letter,ax in zip('ab',(left,right)):
        ax.tick_params(labelsize=9);ax.annotate(letter,xy=(0,1),xycoords='axes fraction',xytext=(-18,16),textcoords='offset points',fontsize=12,fontweight='bold')
    fig.suptitle('A8 · 支援范围敏感性与移动支援对照',fontsize=18,color=ink,y=.98)
    fig.text(.5,.932,'相同初始位姿、UUV 曲线与 89 个扫描窗口；移动方案面向 2300 m 条件设计',ha='center',fontsize=11,color='#586b7d')
    handles=[Line2D([],[],color=teal,label='UUV1'),Line2D([],[],color=orange,label='UUV2'),Line2D([],[],color=purple,label='USV 移动'),Line2D([],[],color=gray,ls='--',label='原驻守'),Line2D([],[],marker='o',ls='',color=teal,label='连续核查通过'),Line2D([],[],marker='X',ls='',color=orange,label='当前方案违约'),Line2D([],[],marker='s',ls='',color=gray,label='初始条件不满足')]
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.894),ncol=7,frameon=False,fontsize=9)
    base=x['strategies']['a7_fixed_station'];mobile=x['strategies']['scheduled_mobile'];maxrange=max(p['maximum_distance_upper_m']for p in mobile['pairs'].values());minimum=min(p['minimum_distance_lower_m']for p in mobile['pairs'].values())
    fig.text(.5,.16,f'USV 航程 {base["usv_length_m"]/1000:.3f} → {mobile["usv_length_m"]/1000:.3f} km（{mobile["usv_length_m"]/base["usv_length_m"]:.2f} 倍）；共同任务仍为 {x["mission_end_s"]/3600:.3f} h',ha='center',fontsize=11,color=ink)
    fig.text(.5,.112,f'移动方案：最大支援距离 ≤ {math.ceil(maxrange*100)/100:.2f} m；最小间距 ≥ {math.floor(minimum*100)/100:.2f} m（要求 25 m）',ha='center',fontsize=11,color=ink)
    fig.text(.5,.071,f'初始位置要求支援范围至少 {x["necessary_conditions"]["initial_required_support_range_m"]:.2f} m；2290 m 下当前两套方案均违约，不等于所有方案无解。',ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.032,'确定性离线二维对照；覆盖下界 99.9039% 保持。理想等待／驻守，未验证真实控制、洋流、能耗、深度、声学与部署回收。',ha='center',fontsize=9,color='#586b7d')
    fig.canvas.draw();boxes=[np.asarray(a.get_window_extent().bounds)*72/fig.dpi for a in(left,right)]
    if np.max(np.abs(boxes[0][1:]-boxes[1][1:]))>1.5:raise ValueError('Panel boxes differ beyond1.5pt')
    if args.qa_script_dir:
        sys.path.insert(0,str(args.qa_script_dir));from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig,json_out=args.output_dir/'alignment.json',require_panel_labels=True,strict=True)
    stem=args.output_dir/'support_sensitivity_a8';fig.savefig(stem.with_suffix('.png'),dpi=300);fig.savefig(stem.with_suffix('.pdf'));fig.savefig(stem.with_suffix('.svg'));plt.close(fig)
    (args.output_dir/'figure_manifest.json').write_text(json.dumps({'source_result_sha256':digest,'plot_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'range_grid_is_categorical':True,'curve_sampling_display_only':True,'outputs':{ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest()for ext in ('png','pdf','svg')}},indent=2)+'\n');print(stem.with_suffix('.png'))


if __name__=='__main__':main()
