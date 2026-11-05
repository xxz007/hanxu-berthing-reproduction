"""A11 actual feedback trajectories and A12 registered-condition outcomes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from matplotlib.patches import PathPatch,Patch
from matplotlib.lines import Line2D
ROOT=Path(__file__).resolve().parents[1]

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

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a11-run-dir','a11-audit-json','a12-study-dir','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path);args=parser.parse_args();read=lambda f:json.loads(f.read_text());x=read(args.a11_run_dir/'result.json');audit=read(args.a11_audit_json);study=read(args.a12_study_dir/'study_summary.json')
    if not audit['passed']or not audit['mission_constraints_certified']or audit['source_result_sha256']!=hashlib.sha256((args.a11_run_dir/'result.json').read_bytes()).hexdigest():raise ValueError('Passed matched A11 audit required')
    if not study['passed']or study['source_nominal_result_sha256']!=audit['source_result_sha256']:raise ValueError('Matched audited A12 study required, including retained failure cases')
    if x['case']!='nominal'or x['effective_step_s']!=.5:raise ValueError('This figure compares the fixed 0.5s nominal baseline')
    args.output_dir.mkdir(parents=True,exist_ok=False)
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Noto Sans CJK SC','Arial','DejaVu Sans'],'font.size':10,'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'none','figure.facecolor':'white','savefig.facecolor':'white'})
    ink='#19334b';teal='#078b8b';orange='#b97825';purple='#7854a2';red='#b14f55';gray='#87929e';colors=(purple,teal,orange)
    def export(fig,axes,name):
        fig.canvas.draw();boxes=np.asarray([a.get_window_extent().bounds for a in axes])*72/fig.dpi
        if np.max(np.abs(boxes[0,1:]-boxes[1,1:]))>1.5:raise ValueError('Comparable panel boxes differ')
        (args.output_dir/(name+'_panels.json')).write_text(json.dumps({'boxes_pt':boxes.tolist(),'passed':True,'tolerance_pt':1.5},indent=2)+'\n')
        if args.qa_script_dir:
            sys.path.insert(0,str(args.qa_script_dir));from audit_panel_alignment import require_matplotlib_panel_alignment
            require_matplotlib_panel_alignment(fig,json_out=args.output_dir/(name+'_alignment.json'),overlay_svg=args.output_dir/(name+'_alignment.svg'),tolerance_pt=1.5,gutter_tolerance_pt=1.5,strict=True)
        stem=args.output_dir/name;fig.savefig(stem.with_suffix('.pdf'))
        for ext in ('png','svg'):fig.savefig(stem.with_suffix('.'+ext),dpi=300)
        plt.close(fig)
    cfg=read(args.a11_run_dir/'config.json');backgrounds={}
    for n,key,d in [('navigation','navigation_sha256',args.navigation_dir),('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir)]:
        f=d/(n+'.geojson')
        if hashlib.sha256(f.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Chart pin changed')
        backgrounds[n]=read(f)['features'][0]['geometry']
    enclosure=read(args.a11_run_dir/'geometry_enclosures.json');z=np.load(args.a11_run_dir/'trajectory.npz');t=z['times'];states=z['states'];ref=z['reference']
    fig,axes=plt.subplots(1,2,figsize=(13,8.6));fig.subplots_adjust(left=.075,right=.98,bottom=.24,top=.81,wspace=.2);left,right=axes
    left.set_facecolor('#f0f1f2');draw(left,backgrounds['navigation'],'#d9efed');draw(left,backgrounds['hazards'],'#8b929a')
    names=('usv','uuv_1','uuv_2');bins=(t//30).astype(int);starts=np.r_[0,np.flatnonzero(np.diff(bins))+1]
    for j,(n,c)in enumerate(zip(names,colors)):
        xy=np.asarray(enclosure['paths'][n]['points']);left.plot(xy[:,0],xy[:,1],color=c,lw=1.0 if j==0 else .6);left.scatter(*states[0,j,:2],color=c,marker='*',s=30,zorder=6)
        error=np.linalg.norm(states[:,j,:2]-ref[:,j,:2],axis=1);maxima=np.maximum.reduceat(error,starts);right.plot(t[starts]/3600,maxima*1000,color=c,lw=.7,label=n.upper().replace('_',''))
    left.set(xlim=(0,5120),ylim=(0,5120),aspect='equal',xlabel='东向 / m',ylabel='北向 / m');right.set_box_aspect(1)
    right.set(xlim=(0,x['mission_end_s']/3600),ylim=(0,1.2),xlabel='共同任务时间 / h',ylabel='30 s 窗内最大位置误差 / mm');right.grid(color='#e6edf0',lw=.6);right.spines[['top','right']].set_visible(False)
    left.set_title('实际轨迹：名义条件下与参考高度重合',loc='left',fontsize=12,pad=15,color=ink);right.set_title('误差来自独立积分，未把状态重置到参考',loc='left',fontsize=12,pad=15,color=ink)
    for letter,ax in zip('ab',axes):ax.annotate(letter,xy=(0,1),xycoords='axes fraction',xytext=(-18,16),textcoords='offset points',fontsize=12,fontweight='bold');ax.tick_params(labelsize=9)
    fig.suptitle('A11 · 受限反馈控制与实际轨迹核查',fontsize=18,color=ink,y=.98);fig.text(.5,.932,'固定 A9 参考、89 个扫描窗口与任务时间；全驱动二维加速度替代模型，更新周期 0.5 s',ha='center',fontsize=11,color='#586b7d')
    handles=[Line2D([],[],color=c,label=n.upper().replace('_',''))for n,c in zip(names,colors)]+[Patch(facecolor='#8b929a',label='海图障碍'),Patch(facecolor='#d9efed',label='安全导航域')];fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.894),ncol=5,frameon=False,fontsize=9)
    pairs=audit['distance_pairs'];mx=max(v['maximum_distance_upper_m']for k,v in pairs.items()if k.startswith('usv__'));mn=min(v['minimum_distance_lower_m']for v in pairs.values())
    fig.text(.5,.16,f'独立核查：支援距离 ≤ {math.ceil(mx*100)/100:.2f} m；平台间距 ≥ {math.floor(mn*100)/100:.2f} m；原 50 m 障碍裕度保持',ha='center',fontsize=11,color=ink)
    fig.text(.5,.112,f'实际轨迹保守覆盖下界 {x["geometry"]["coverage_lower_ratio"]*100:.5f}%（要求 99.9%）；目标面积与扫描开启时段不变',ha='center',fontsize=11,color=ink)
    fig.text(.5,.071,'名义仿真最大位置误差约 1.06 mm；理想地速测量、精确模型与前馈条件下的数值结果，不代表实艇精度。',ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.032,'水平速度方向与航向独立可控；未建立欠驱动水动力学、推力或能耗模型，也未复现论文 MPC。',ha='center',fontsize=9,color='#586b7d');export(fig,axes,'tracking_control_a11')
    rows=study['cases'];labels={'nominal':'名义条件','initial_offset':'初始偏差','drift_005':'0.05 m/s 漂流','sensor_010':'0.1 m 测量偏差','combined':'组合误差','sensor_5m':'5 m 测量偏差','current_over_limit':'超速度漂流'}
    fig,axes=plt.subplots(1,2,figsize=(13,8.6));fig.subplots_adjust(left=.14,right=.98,bottom=.24,top=.81,wspace=.2);left,right=axes
    styles={'certified_pass':('o',teal),'witnessed_violation':('X',red),'unresolved_bounds':('D',gray)}
    for i,row in enumerate(rows):
        a=read(Path(row['audit_json']));pairs=a['distance_pairs'];sep=list(pairs.values());support=[v for k,v in pairs.items()if k.startswith('usv__')]
        status=[('certified_pass'if a['independent_coverage_lower_ratio']>=.999 else'witnessed_violation'if a['independent_coverage_upper_ratio']<.999 else'unresolved_bounds'),('certified_pass'if min(a['independent_hazard_clearance_lower_m'].values())>=50 and all(a['navigation_enclosures_passed'].values())else'unresolved_bounds'),('certified_pass'if min(v['minimum_distance_lower_m']for v in sep)>=25 else'witnessed_violation'if min(v['minimum_distance_witness_m']for v in sep)<25 else'unresolved_bounds'),('certified_pass'if max(v['maximum_distance_upper_m']for v in support)<=2300 else'witnessed_violation'if max(v['maximum_distance_witness_m']for v in support)>2300 else'unresolved_bounds'),row['constraint_outcome']]
        for j,s in enumerate(status):marker,color=styles[s];left.scatter(j,i,marker=marker,color=color,s=80)
        right.barh(i,row['maximum_position_error_m'],height=.5,color=styles[row['constraint_outcome']][1]);right.text(row['maximum_position_error_m']*1.2,i,f'{row["maximum_position_error_m"]:.3g}',va='center',fontsize=9)
    for ax in axes:ax.set_box_aspect(1);ax.set_ylim(len(rows)-.5,-.5);ax.tick_params(labelsize=9)
    left.set(xlim=(-.6,4.6),xticks=range(5),xticklabels=['覆盖\n≥99.9%','障碍\n≥50 m','间距\n≥25 m','支援\n≤2300 m','整体'],yticks=range(len(rows)),yticklabels=[labels[r['case']]for r in rows]);left.tick_params(axis='x',length=0,pad=8);left.grid(axis='y',color='#e6edf0',lw=.6)
    right.set_xscale('log');right.set(xlim=(1e-4,1e5),xticks=[1e-4,1e-2,1,1e2,1e4],xticklabels=['0.0001','0.01','1','100','10000'],yticks=range(len(rows)),yticklabels=[],xlabel='全任务最大位置误差 / m（对数轴）');right.grid(axis='x',color='#e6edf0',lw=.6);right.spines[['top','right']].set_visible(False)
    for letter,ax in zip('ab',axes):ax.annotate(letter,xy=(0,1),xycoords='axes fraction',xytext=(-18,16),textcoords='offset points',fontsize=12,fontweight='bold')
    left.set_title('按同一门槛核查实际轨迹',loc='left',fontsize=12,pad=15,color=ink);right.set_title('误差幅度与约束结果一起报告',loc='left',fontsize=12,pad=15,color=ink)
    fig.suptitle('A12 · 扰动与测量误差的适用边界',fontsize=18,color=ink,y=.98);fig.text(.5,.932,'预登记 7 个确定性条件；地图、参考、控制器、速度／加速度上限和任务门槛保持',ha='center',fontsize=11,color='#586b7d')
    handles=[Line2D([],[],marker=m,ls='',color=c,label=label)for (m,c),label in zip(styles.values(),('认证通过','存在违约见证','未认证'))];fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.894),ncol=3,frameon=False,fontsize=10)
    counts={s:sum(r['constraint_outcome']==s for r in rows)for s in styles};fig.text(.5,.16,f'整体：{counts["certified_pass"]} 个条件通过，{counts["witnessed_violation"]} 个有违约见证，{counts["unresolved_bounds"]} 个未认证；失败案例完整保留',ha='center',fontsize=11,color=ink)
    fig.text(.5,.112,'覆盖按实际扫描轨迹重算；漂流进入位置方程，测量偏差进入反馈，均未用参考轨迹代替实际状态。',ha='center',fontsize=10,color=ink)
    fig.text(.5,.071,'这些强度是工程测试设定，并非当地实测水流或设备规格；各条件只有一次确定性仿真，不作统计显著性推断。',ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.032,'全驱动二维替代模型；理想地速测量、恒定漂流，无真实声学、深度、部署回收及实艇验证。',ha='center',fontsize=9,color='#586b7d');export(fig,axes,'tracking_robustness_a12')
    (args.output_dir/'figure_manifest.json').write_text(json.dumps({'a11_source_result_sha256':hashlib.sha256((args.a11_run_dir/'result.json').read_bytes()).hexdigest(),'outputs':{f.name:hashlib.sha256(f.read_bytes()).hexdigest()for f in args.output_dir.iterdir()if f.suffix in('.png','.pdf','.svg')}},indent=2)+'\n')
    print(args.output_dir)


if __name__=='__main__':main()
