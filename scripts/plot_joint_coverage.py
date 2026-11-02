"""Plot independently audited A6 individual routes and two repaired short scans."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece
from run_motion_feasibility import sample_enclosure
from chart_geometry import Geos


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','repair-dir','prepared-dir','navigation-dir','audit-json','repair-audit-json','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path)
    args=parser.parse_args();x=json.loads((args.run_dir/'result.json').read_text());audit=json.loads(args.audit_json.read_text());cfg=json.loads((args.run_dir/'config.json').read_text())
    result_hash=hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest()
    if not audit['passed']or not x['complete_platform_routes_found']or audit['source_result_sha256']!=result_hash:raise ValueError('Only matching audited complete routes can be plotted')
    repair=json.loads((args.repair_dir/'result.json').read_text());repair_audit=json.loads(args.repair_audit_json.read_text());repair_hash=hashlib.sha256((args.repair_dir/'result.json').read_bytes()).hexdigest()
    if not repair_audit['passed']or repair_audit['route_source_result_sha256']!=result_hash or repair_audit['repair_source_result_sha256']!=repair_hash:raise ValueError('Matching audited repair policy required')
    shapes={}
    for name in ('searchable','hazards'):
        p=args.prepared_dir/(name+'.geojson')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg['A1_geometry_sha256'][name]:raise ValueError('Changed figure geometry')
        shapes[name]=json.loads(p.read_text())['features'][0]['geometry']
    p=args.navigation_dir/'navigation.geojson'
    if hashlib.sha256(p.read_bytes()).hexdigest()!=cfg['expected_navigation_sha256']:raise ValueError('Changed navigation')
    shapes['navigation']=json.loads(p.read_text())['features'][0]['geometry']
    args.output_dir.mkdir(parents=True,exist_ok=False)
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.path import Path as MplPath
    from matplotlib.patches import PathPatch,Patch
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Noto Sans CJK SC','Arial','DejaVu Sans'],
                         'font.size':10,'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'none','figure.facecolor':'white','savefig.facecolor':'white'})
    colors={'uuv_1':'#078b8b','uuv_2':'#bb7726','usv':'#7854a2'};ink='#19334b'
    def draw(ax,shape,color):
        kind=shape['type']
        if kind=='GeometryCollection':
            for member in shape['geometries']:draw(ax,member,color)
        elif kind=='MultiPolygon':
            for poly in shape['coordinates']:draw(ax,{'type':'Polygon','coordinates':poly},color)
        elif kind=='Polygon':
            vertices=[];codes=[]
            for i,ring in enumerate(shape['coordinates']):
                if not ring:continue
                xy=np.asarray(ring);signed=np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])
                if(signed>0)!=(i==0):xy=xy[::-1]
                vertices.extend(xy);codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
            if vertices:ax.add_patch(PathPatch(MplPath(vertices,codes),facecolor=color,edgecolor='none'))
        elif kind in ('LineString','MultiLineString'):
            for line in ([shape['coordinates']]if kind=='LineString'else shape['coordinates']):
                xy=np.asarray(line)
                if len(xy):ax.plot(xy[:,0],xy[:,1],color=color,lw=.7)
        elif kind in ('Point','MultiPoint'):
            xy=np.asarray([shape['coordinates']]if kind=='Point'else shape['coordinates'])
            if len(xy):ax.scatter(xy[:,0],xy[:,1],color=color,s=4)
    local=[1280,1960,1720,2400]
    bounds_list=([0,0,5120,5120],local);viewports=[{'type':'Polygon','coordinates':[[[a,b],[c,b],[c,d],[a,d],[a,b]]]}for a,b,c,d in bounds_list]
    route_xy={name:np.asarray(sample_enclosure([Piece(**p)for p in plan['pieces']],.02)[0])for name,plan in x['platforms'].items()}
    # Physically clip display geometry. PDF clip paths alone leave invisible
    # out-of-panel strokes in the file and make rendered-collision QA ambiguous.
    with Geos()as g:
        backgrounds=[{name:g.operation('intersection',shape,window)for name,shape in shapes.items()}for window in viewports]
        routes={name:[g.operation('intersection',{'type':'LineString','coordinates':xy.tolist()},window)for window in viewports]for name,xy in route_xy.items()}
    def draw_route(ax,shape,color,width,zorder=4):
        kind=shape['type']
        if kind=='GeometryCollection':
            for member in shape['geometries']:draw_route(ax,member,color,width,zorder)
        elif kind=='MultiLineString':
            for line in shape['coordinates']:draw_route(ax,{'type':'LineString','coordinates':line},color,width,zorder)
        elif kind=='LineString':
            xy=np.asarray(shape['coordinates'])
            if len(xy):ax.plot(xy[:,0],xy[:,1],color=color,lw=width,zorder=zorder)
    fig,axes=plt.subplots(1,2,figsize=(13,8.6));fig.subplots_adjust(left=.065,right=.98,bottom=.24,top=.81,wspace=.17)
    for letter,ax,bounds,title,background in zip('ab',axes,bounds_list,('完整个体路线与按需补扫','原短扫描段及实际曲线连接（局部）'),backgrounds):
        ax.set_facecolor('#f0f1f2');draw(ax,background['navigation'],'#d9efed');draw(ax,background['searchable'],'#dce8f3');draw(ax,background['hazards'],'#8b929a')
        ax.set(xlim=(bounds[0],bounds[2]),ylim=(bounds[1],bounds[3]),xlabel='东向 / m',ylabel='北向 / m',aspect='equal')
        ax.tick_params(labelsize=9);ax.ticklabel_format(useOffset=False,style='plain');ax.set_title(title,loc='left',fontsize=12,pad=15,color=ink)
        ax.annotate(letter,xy=(0,1),xycoords='axes fraction',xytext=(-18,16),textcoords='offset points',fontweight='bold',fontsize=12)
    for name,plan in x['platforms'].items():
        xy=route_xy[name];draw_route(axes[0],routes[name][0],colors[name],.65 if name!='usv'else 1.3)
        axes[0].scatter(*xy[0],color=colors[name],marker='o',s=17,zorder=6);axes[0].scatter(*xy[-1],color=colors[name],marker='*',s=35,zorder=6)
        if name=='uuv_1':draw_route(axes[1],routes[name][1],colors[name],1.1)
    for record in repair['selected_repairs']:
        leg=x['platforms'][record['platform']]['legs'][record['source_transit_leg_index']];xy,_=sample_enclosure([Piece(**p)for p in leg['pieces']],.02);xy=np.asarray(xy)
        axes[0].plot(xy[:,0],xy[:,1],color='#244f7b',lw=1.8,zorder=8)
    for v in x['platforms']['uuv_1']['selected_scans']:
        if v['task_id']not in ('scan_018','scan_019'):continue
        old=np.asarray(v['original_points']);new=np.asarray(v['points'])
        axes[1].plot(old[:,0],old[:,1],color='#6b7786',ls='--',lw=1.8,zorder=7)
        axes[1].plot(new[:,0],new[:,1],color='#b74d49',lw=2.3,zorder=8)
        axes[1].scatter(new[:,0],new[:,1],s=15,color='#b74d49',zorder=9)
    fig.suptitle('A6 · 全任务连续曲线与按需补扫',fontsize=18,color=ink,y=.98)
    total=sum(len(x['platforms'][name]['task_ids'])for name in ('uuv_1','uuv_2'))
    fig.text(.5,.931,f'保留原 {total} 条任务并增加 {repair["explicit_repair_task_count"]} 条明确补扫；原目标 8.767 km²、UUV / USV 转弯半径 20 / 40 m、障碍裕度 50 m',ha='center',fontsize=11,color='#586b7d')
    fig.legend(handles=[Patch(facecolor='#8b929a',label='实体障碍'),Line2D([],[],color=colors['uuv_1'],label='UUV1'),Line2D([],[],color=colors['uuv_2'],label='UUV2'),
                        Line2D([],[],color=colors['usv'],label='USV 独立路线'),Line2D([],[],color='#244f7b',lw=1.8,label='明确补扫段'),Line2D([],[],color='#6b7786',ls='--',label='原短扫描段'),Line2D([],[],color='#b74d49',label='调整后的短扫描段')],
               loc='upper center',bbox_to_anchor=(.5,.90),ncol=7,frameon=False,fontsize=9)
    first=x['platforms']['uuv_1'];second=x['platforms']['uuv_2'];coverage=x['coverage']
    fig.text(.5,.16,f'UUV1：{len(first["task_ids"])} 条任务 / {first["length_m"]/1000:.2f} km　　UUV2：{len(second["task_ids"])} 条任务 / {second["length_m"]/1000:.2f} km',ha='center',fontsize=11,color=ink)
    fig.text(.5,.112,f'原 {total} 段：{coverage["scan_only_coverage_ratio"]*100:.4f}%　　加 {repair["explicit_repair_task_count"]} 条补扫：≥{repair["coverage_lower_ratio"]*100:.4f}%　　全转场 ON 对照：≥{coverage["transit_scan_ON_lower_ratio"]*100:.4f}%',ha='center',fontsize=11,color=ink)
    fig.text(.5,.071,'覆盖按完整原目标重新计算，理想水平足迹半径 50 m；补扫利用既有曲线，其余转场扫描 OFF。',ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.032,'UUV2 初始位置因短任务先行而改变；预部署／回收、协同时钟和连续间距、真实船体／动力学／声学尚未验收。',ha='center',fontsize=9,color='#586b7d')
    fig.canvas.draw()
    rectangles=[np.asarray(ax.get_window_extent().bounds)*72/fig.dpi for ax in axes]
    # Self-contained minimum gate for users without the optional skill helper.
    differences=np.abs(rectangles[0][1:]-rectangles[1][1:])
    if np.max(differences)>1.5:raise ValueError('Rendered map panels are not aligned within1.5pt')
    (args.output_dir/'local_alignment.json').write_text(json.dumps({'passed':True,'rectangles_pt':[r.tolist()for r in rectangles],'maximum_y_width_height_difference_pt':float(np.max(differences))},indent=2)+'\n')
    if args.qa_script_dir:
        sys.path.insert(0,str(args.qa_script_dir));from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig,json_out=args.output_dir/'alignment.json',require_panel_labels=True,strict=True)
    stem=args.output_dir/'joint_coverage_a6'
    fig.savefig(stem.with_suffix('.png'),dpi=300);fig.savefig(stem.with_suffix('.pdf'));fig.savefig(stem.with_suffix('.svg'));plt.close(fig)
    manifest={'source_result_sha256':result_hash,'repair_result_sha256':repair_hash,'audit_sha256':hashlib.sha256(args.audit_json.read_bytes()).hexdigest(),'repair_audit_sha256':hashlib.sha256(args.repair_audit_json.read_bytes()).hexdigest(),'plot_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'local_display_bounds_m':local,'display_geometry_clipped_to_axes':True,'all_source_tasks_in_left_panel':total,'geometry_downsampling':False,'cooperative_clock_claimed':False,
              'output_sha256':{ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest()for ext in ('png','pdf','svg')}}
    (args.output_dir/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(stem.with_suffix('.png'))


if __name__=='__main__':main()
