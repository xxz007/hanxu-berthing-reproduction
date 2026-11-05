"""A9 audited route map and categorical support-range sensitivity comparison."""
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
    for name in ('run-dir','a8-run-dir','audit-json','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path);args=parser.parse_args();read=lambda p:json.loads(p.read_text())
    x=read(args.run_dir/'result.json');old=read(args.a8_run_dir/'result.json');audit=read(args.audit_json);cfg=read(args.run_dir/'config.json')
    digest=hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()
    if not audit['passed']or audit['source_result_sha256']!=digest:raise ValueError('Matching independent A9 audit required')
    canon=lambda v:hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    if canon({k:v for k,v in old.items()if k!='total_computation_seconds'})!=cfg['source_a8_scientific_sha256']:raise ValueError('A8 baseline changed')
    backgrounds={}
    for name,key,d in [('navigation','navigation_sha256',args.navigation_dir),('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir)]:
        p=d/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Chart pin changed')
        backgrounds[name]=read(p)['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False)
    import matplotlib;matplotlib.use('Agg')
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
    fig,axes=plt.subplots(1,2,figsize=(13,8.6));fig.subplots_adjust(left=.065,right=.98,bottom=.24,top=.81,wspace=.17)
    for i,(ax,value,color)in enumerate(zip(axes,(old,x),(purple,'#b14f55'))):
        ax.set_facecolor('#f0f1f2');draw(ax,backgrounds['navigation'],'#d9efed');draw(ax,backgrounds['searchable'],'#dce8f3');draw(ax,backgrounds['hazards'],'#8b929a')
        for name,c in [('uuv_1',teal),('uuv_2',orange)]:
            xy=np.asarray(sample_enclosure([Piece(**p)for p in x['uuv_platforms'][name]['pieces']],.02)[0]);ax.plot(xy[:,0],xy[:,1],color=c,lw=.55,alpha=.65)
        for leg in value['mobile_usv']['legs']:
            xy=np.asarray(sample_enclosure([Piece(**p)for p in leg['pieces']],.02)[0]);ax.plot(xy[:,0],xy[:,1],color=color,lw=1.8,zorder=5);ax.scatter(*xy[-1],s=24,facecolors='white',edgecolors=color,zorder=6)
        ax.scatter(*x['mobile_usv']['initial_pose'][:2],s=65,marker='*',color=color,zorder=7)
        ax.set(xlim=(0,5120),ylim=(0,5120),aspect='equal',xlabel='东向 / m',ylabel='北向 / m')
        ax.set_title(('A8：窗口内优先缩小支援距离','A9：延长驻守，减少转场')[i],loc='left',fontsize=12,pad=15,color=ink)
        ax.annotate('ab'[i],xy=(0,1),xycoords='axes fraction',xytext=(-18,16),textcoords='offset points',fontsize=12,fontweight='bold');ax.tick_params(labelsize=9)
    fig.suptitle('A9 · 相同任务与约束下，减少 USV 支援航程',fontsize=18,color=ink,y=.98)
    fig.text(.5,.932,'固定初始位姿、UUV 曲线与速度、89 个扫描窗口；支援范围 2300 m / 安全间距 25 m / 障碍裕度 50 m',ha='center',fontsize=10,color='#586b7d')
    handles=[Patch(facecolor='#8b929a',label='海图障碍'),Patch(facecolor='#d9efed',label='安全导航域'),Line2D([],[],color=teal,label='UUV1'),Line2D([],[],color=orange,label='UUV2'),Line2D([],[],color=purple,label='A8 USV'),Line2D([],[],color='#b14f55',label='A9 USV'),Line2D([],[],marker='o',markerfacecolor='white',ls='',color=gray,label='驻点')]
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.894),ncol=7,frameon=False,fontsize=9)
    c=x['comparison'];pairs=x['strategies']['scheduled_mobile']['pairs'];maximum=max(p['maximum_distance_upper_m']for p in pairs.values());minimum=min(p['minimum_distance_lower_m']for p in pairs.values())
    fig.text(.5,.16,f'USV 航程 {c["source_a8_length_m"]/1000:.3f} → {c["optimized_length_m"]/1000:.3f} km（减少 {c["length_saved_ratio"]*100:.2f}%）；转场 {c["source_a8_transfer_count"]} → {c["optimized_transfer_count"]} 次',ha='center',fontsize=12,color=ink)
    fig.text(.5,.112,f'A9 连续核查：最大支援距离 ≤ {math.ceil(maximum*100)/100:.2f} m；最小 USV–UUV 间距 ≥ {math.floor(minimum*100)/100:.2f} m',ha='center',fontsize=11,color=ink)
    fig.text(.5,.071,f'共同任务 {x["mission_end_s"]/3600:.3f} h；理想覆盖下界 {x["coverage_lower_ratio_unchanged"]*100:.4f}% 保持。航程减少不等于已验证节能。',ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.032,'有限网格、离线前视候选改进，未证明全局最优；理想驻守，未验证真实控制、洋流、深度、声学及部署回收。',ha='center',fontsize=9,color='#586b7d')
    fig.canvas.draw();boxes=[np.asarray(a.get_window_extent().bounds)*72/fig.dpi for a in axes]
    if np.max(np.abs(boxes[0][1:]-boxes[1][1:]))>1.5:raise ValueError('Panel boxes differ beyond 1.5pt')
    (args.output_dir/'panel_boxes.json').write_text(json.dumps({'boxes_pt':[b.tolist()for b in boxes],'tolerance_pt':1.5,'passed':True},indent=2)+'\n')
    if args.qa_script_dir:
        sys.path.insert(0,str(args.qa_script_dir));from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig,json_out=args.output_dir/'alignment.json',overlay_svg=args.output_dir/'alignment.svg',tolerance_pt=1.5,gutter_tolerance_pt=1.5,strict=True)
    stem=args.output_dir/'support_optimization_a9'
    fig.savefig(stem.with_suffix('.pdf'))
    for ext in ('png','svg'):fig.savefig(stem.with_suffix('.'+ext),dpi=300)
    plt.close(fig)
    (args.output_dir/'figure_manifest.json').write_text(json.dumps({'source_result_sha256':digest,'curve_sampling_display_only':True,'outputs':{ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest()for ext in ('png','pdf','svg')}},indent=2)+'\n');print(stem.with_suffix('.png'))


if __name__=='__main__':main()
