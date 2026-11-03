"""A7 audited whole-route map and common-clock support-distance comparison."""
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
from berthing_planner.curved_timing import CurvedClock
from chart_geometry import Geos
from run_motion_feasibility import sample_enclosure


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','a6-run-dir','audit-json','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path)
    args=parser.parse_args();x=json.loads((args.run_dir/'result.json').read_text());audit=json.loads(args.audit_json.read_text());cfg=json.loads((args.run_dir/'config.json').read_text());source=json.loads((args.a6_run_dir/'result.json').read_text())
    source_hash=hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()
    if not audit['passed']or audit['source_result_sha256']!=source_hash:raise ValueError('Matching independent A7 audit required')
    canon=lambda v:hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()).hexdigest()
    if canon({k:v for k,v in source.items()if k!='total_computation_seconds'})!=cfg['source_a6_scientific_sha256']:raise ValueError('A6 plotting source changed')
    shapes={}
    for name,key,directory in [('searchable','target_geometry_sha256',args.prepared_dir),('hazards','hazards_geometry_sha256',args.prepared_dir),('navigation','navigation_sha256',args.navigation_dir)]:
        p=directory/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg[key]:raise ValueError('Plot geometry changed')
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False);window={'type':'Polygon','coordinates':[[[0,0],[5120,0],[5120,5120],[0,5120],[0,0]]]}
    route_xy={name:sample_enclosure([Piece(**p)for p in body['pieces']],.02)[0]for name,body in x['platforms'].items()};old_xy=sample_enclosure([Piece(**p)for p in source['platforms']['usv']['pieces']],.02)[0]
    with Geos()as g:
        backgrounds={name:g.operation('intersection',shape,window)for name,shape in shapes.items()};routes={name:g.operation('intersection',{'type':'LineString','coordinates':xy},window)for name,xy in route_xy.items()};old_route=g.operation('intersection',{'type':'LineString','coordinates':old_xy},window)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.path import Path as MplPath
    from matplotlib.patches import PathPatch,Patch
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Noto Sans CJK SC','Arial','DejaVu Sans'],'font.size':10,'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'none','figure.facecolor':'white','savefig.facecolor':'white'})
    colors={'uuv_1':'#078b8b','uuv_2':'#bb7726','usv':'#7854a2'};ink='#19334b';gray='#87929e'
    def draw(ax,shape,face,edge='none',width=.7,style='-'):
        kind=shape['type']
        if kind=='GeometryCollection':
            for member in shape['geometries']:draw(ax,member,face,edge,width,style)
        elif kind=='MultiPolygon':
            for p in shape['coordinates']:draw(ax,{'type':'Polygon','coordinates':p},face,edge,width,style)
        elif kind=='Polygon':
            vertices=[];codes=[]
            for i,ring in enumerate(shape['coordinates']):
                if not ring:continue
                xy=np.asarray(ring);signed=np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])
                if(signed>0)!=(i==0):xy=xy[::-1]
                vertices.extend(xy);codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
            if vertices:ax.add_patch(PathPatch(MplPath(vertices,codes),facecolor=face,edgecolor=edge,lw=width))
        elif kind=='MultiLineString':
            for line in shape['coordinates']:draw(ax,{'type':'LineString','coordinates':line},face,edge,width,style)
        elif kind=='LineString':
            xy=np.asarray(shape['coordinates'])
            if len(xy):ax.plot(xy[:,0],xy[:,1],color=edge if edge!='none'else face,lw=width,ls=style)
        elif kind in('Point','MultiPoint'):
            xy=np.asarray([shape['coordinates']]if kind=='Point'else shape['coordinates'])
            if len(xy):ax.scatter(xy[:,0],xy[:,1],color=face,s=4)
    fig,(left,right)=plt.subplots(1,2,figsize=(13,8.6));fig.subplots_adjust(left=.065,right=.98,bottom=.24,top=.81,wspace=.17)
    left.set_facecolor('#f0f1f2');draw(left,backgrounds['navigation'],'#d9efed');draw(left,backgrounds['searchable'],'#dce8f3');draw(left,backgrounds['hazards'],'#8b929a')
    draw(left,old_route,gray,gray,.9,'--')
    for name,shape in routes.items():
        draw(left,shape,colors[name],colors[name],1.7 if name=='usv'else .65)
        xy=route_xy[name];left.scatter(*xy[0],s=17,color=colors[name],zorder=6);left.scatter(*xy[-1],s=45,marker='D'if name=='usv'else'*',color=colors[name],zorder=6)
    left.set(xlim=(0,5120),ylim=(0,5120),xlabel='东向 / m',ylabel='北向 / m',aspect='equal');left.set_title('保持初始位姿，USV 转场后驻守',loc='left',fontsize=12,pad=15,color=ink)
    clocks={n:CurvedClock(p['pieces'],p['profile'],p['departure_s'])for n,p in x['platforms'].items()};old={n:CurvedClock(p['pieces'],p['profile'])for n,p in source['platforms'].items()};end=x['mission_end_s'];naive=json.loads((args.run_dir/'naive_baseline.json').read_text())
    times={0.,end,*np.arange(0,end,30).tolist()}
    for pairs in (x['distance_pairs'],naive['pairs']):
        for p in pairs.values():times.update([p['minimum_witness_time_s'],p['maximum_witness_time_s']])
    times=np.asarray(sorted(times));curves={}
    for name in('uuv_1','uuv_2'):
        values=[math.dist(clocks['usv'].state(float(t))['position_m'],clocks[name].state(float(t))['position_m'])for t in times];oldvalues=[math.dist(old['usv'].state(float(t))['position_m'],old[name].state(float(t))['position_m'])for t in times]
        right.plot(times/3600,oldvalues,color=gray,lw=.8,ls='--',alpha=.7);right.plot(times/3600,values,color=colors[name],lw=1.2);curves[name]={'new':values,'naive':oldvalues}
    right.axhline(cfg['support_range_m'],color='#b74d49',ls='--',lw=1);right.axhline(cfg['minimum_separation_m'],color='#b74d49',ls=':',lw=.8)
    right.set(xlim=(0,end/3600),ylim=(0,3400),xlabel='共同任务时间 / h',ylabel='USV 至 UUV 距离 / m');right.set_box_aspect(1);right.set_title('全过程支援距离与原路线对照',loc='left',fontsize=12,pad=15,color=ink)
    for letter,ax in zip('ab',(left,right)):
        ax.tick_params(labelsize=9);ax.annotate(letter,xy=(0,1),xycoords='axes fraction',xytext=(-18,16),textcoords='offset points',fontweight='bold',fontsize=12)
    minimum=min(p['minimum_distance_lower_m']for p in x['distance_pairs'].values());maximum=max(p['maximum_distance_upper_m']for name,p in x['distance_pairs'].items()if'usv'in name)
    fig.suptitle('A7 · 共同任务时钟与 USV 支援',fontsize=18,color=ink,y=.98)
    fig.text(.5,.931,'A6 初始位姿、UUV 曲线与 89 个扫描窗口保持；安全约束按完整时间区间独立核查',ha='center',fontsize=11,color='#586b7d')
    fig.legend(handles=[Patch(facecolor='#8b929a',label='实体障碍'),Line2D([],[],color=colors['uuv_1'],label='UUV1'),Line2D([],[],color=colors['uuv_2'],label='UUV2'),Line2D([],[],color=colors['usv'],label='USV 转场／驻点'),Line2D([],[],color=gray,ls='--',label='旧 USV 对照'),Line2D([],[],color='#b74d49',ls='--',label='3000 m 上限')],loc='upper center',bbox_to_anchor=(.5,.90),ncol=6,frameon=False,fontsize=9)
    surface=x['platforms']['usv'];wait=x['platforms']['uuv_1']['holds'][-1]['t1_s']-x['platforms']['uuv_1']['holds'][-1]['t0_s']
    fig.text(.5,.16,f'任务 {end/3600:.3f} h　 USV 转场 {surface["length_m"]:.2f} m / {surface["motion_finish_s"]/60:.2f} min　 UUV1 完成后等待 {wait/3600:.3f} h',ha='center',fontsize=11,color=ink)
    fig.text(.5,.112,f'最小间距 ≥ {math.floor(minimum*100)/100:.2f} m（要求 25 m）　 最大支援距离 ≤ {math.ceil(maximum*100)/100:.2f} m（要求 3000 m）',ha='center',fontsize=11,color=ink)
    fig.text(.5,.071,'覆盖下界 99.9039% 保持；原 A6 直接出发对照约 5 m 近距、3.225 km 超距，未通过。',ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.032,'图中曲线采样用于展示；验收使用连续区间证书。理想驻守与水平几何范围，不含部署／回收、真实水深／动力学／声学。',ha='center',fontsize=9,color='#586b7d')
    fig.canvas.draw();boxes=[np.asarray(ax.get_window_extent().bounds)*72/fig.dpi for ax in(left,right)]
    if np.max(np.abs(boxes[0][1:]-boxes[1][1:]))>1.5:raise ValueError('Panel geometry not aligned within1.5pt')
    if args.qa_script_dir:
        sys.path.insert(0,str(args.qa_script_dir));from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig,json_out=args.output_dir/'alignment.json',require_panel_labels=True,strict=True)
    stem=args.output_dir/'curved_cooperation_a7';fig.savefig(stem.with_suffix('.png'),dpi=300);fig.savefig(stem.with_suffix('.pdf'));fig.savefig(stem.with_suffix('.svg'));plt.close(fig)
    (args.output_dir/'figure_manifest.json').write_text(json.dumps({'source_result_sha256':source_hash,'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'plotted_time_points':len(times),'sampled_curves_are_display_only':True,'output_sha256':{ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest()for ext in('png','pdf','svg')}},indent=2)+'\n');print(stem.with_suffix('.png'))


if __name__=='__main__':main()
