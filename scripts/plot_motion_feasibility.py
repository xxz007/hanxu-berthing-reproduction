"""Plot an audited A4 local workspace correction and scalar speed laws."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.curves import Piece
from berthing_planner.motion_feasibility import profile_state
from chart_geometry import Geos


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--audit-json',type=Path,required=True)
    parser.add_argument('--comparison-run-dir',type=Path,required=True)
    parser.add_argument('--comparison-audit-json',type=Path,required=True)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path)
    args=parser.parse_args()
    results=[]
    for directory,audit in [(args.run_dir,args.audit_json),(args.comparison_run_dir,args.comparison_audit_json)]:
        x=json.loads((directory/'result.json').read_text());provenance=json.loads((directory/'provenance.json').read_text())
        if not json.loads(audit.read_text())['passed']:raise ValueError('Only independently audited results can be plotted')
        if hashlib.sha256((directory/'result.json').read_bytes()).hexdigest()!=provenance['result_sha256']:raise ValueError('Changed result')
        results.append(x)
    first,second=results;patch=first['local_patch'];cfg=json.loads((args.run_dir/'config.json').read_text())
    nominal=next(r for r in first['radius_sweep']if r['minimum_radius_m']==cfg['nominal_uuv_minimum_radius_m'])
    file=args.prepared_dir/'searchable.geojson'
    if hashlib.sha256(file.read_bytes()).hexdigest()!=first['target_geometry_sha256']:raise ValueError('Changed map')
    target=json.loads(file.read_text())['features'][0]['geometry']
    a,b=patch['source_scans'];xend=patch['new_scan_end'][0];ymid=(a['points'][0][1]+b['points'][0][1])/2
    width=250.0;height=width*3.7/4.75
    bounds=[xend-130,ymid-height/2,xend+120,ymid+height/2]
    viewport={'type':'Polygon','coordinates':[[[bounds[0],bounds[1]],[bounds[2],bounds[1]],[bounds[2],bounds[3]],[bounds[0],bounds[3]],[bounds[0],bounds[1]]]]}
    with Geos()as g:cropped=g.operation('intersection',target,viewport)
    args.output_dir.mkdir(parents=True,exist_ok=False)
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import PathPatch,Patch
    from matplotlib.path import Path as MplPath
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family':'sans-serif','font.sans-serif':['Noto Sans CJK SC','Arial','DejaVu Sans'],
                         'font.size':10,'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'none',
                         'figure.facecolor':'white','savefig.facecolor':'white'})
    ink,green,red,gray,purple='#19334b','#078b8b','#b44f62','#7d8c98','#744c99'
    fig,axes=plt.subplots(1,2,figsize=(13.8,7.8))
    fig.subplots_adjust(left=.075,right=.975,bottom=.31,top=.80,wspace=.24)
    for i,ax in enumerate(axes):
        ax.set_box_aspect(3.7/4.75);ax.spines[['top','right']].set_visible(False)
        ax.grid(color='#e5eaed',lw=.5);ax.tick_params(labelsize=10,colors='#526676')
        ax.text(-.04,1.065,chr(97+i),transform=ax.transAxes,fontweight='bold',fontsize=12,color=ink,va='bottom')
    left,right=axes;left.set_facecolor('#f3f0eb')
    def draw_geometry(shape):
        if shape['type']=='GeometryCollection':
            for g in shape['geometries']:draw_geometry(g)
        elif shape['type']=='MultiPolygon':
            for p in shape['coordinates']:draw_geometry({'type':'Polygon','coordinates':p})
        elif shape['type']=='Polygon':
            vertices,codes=[],[]
            for i,ring in enumerate(shape['coordinates']):
                xy=np.asarray(ring);signed=np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])
                if(signed>0)!=(i==0):xy=xy[::-1]
                vertices.extend(xy);codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
            if vertices:left.add_patch(PathPatch(MplPath(vertices,codes),facecolor='#e2eff4',edgecolor='#718998',lw=.8))
    draw_geometry(cropped)
    # Clip the old straight scans to the local display window for drawing.
    for leg in patch['source_scans']:
        points=np.asarray(leg['points'],dtype=float);points[:,0]=np.clip(points[:,0],bounds[0],bounds[2])
        left.plot(points[:,0],points[:,1],color=gray,ls='--',lw=1.4)
    points=np.asarray(patch['sampled_points']);left.plot(points[:,0],points[:,1],color=green,lw=2)
    original=a['points'][-1];heading=math.atan2(a['points'][-1][1]-a['points'][0][1],a['points'][-1][0]-a['points'][0][0])
    radius=cfg['nominal_uuv_minimum_radius_m']
    turn=Piece('arc',pose=(*original,heading),length=math.pi*radius/2,curvature=math.copysign(1/radius,patch['pieces'][1]['curvature']))
    bad=np.asarray([turn.evaluate(t)[:2]for t in np.linspace(0,1,60)])
    left.plot(bad[:,0],bad[:,1],color=red,ls=':',lw=1.7)
    for p in (a['points'][-1],b['points'][0]):left.scatter(*p,marker='x',color=red,s=38,zorder=8)
    for p in (patch['new_scan_end'],patch['new_next_scan_start']):left.scatter(*p,color=green,s=22,zorder=8)
    left.set(xlim=(bounds[0],bounds[2]),ylim=(bounds[1],bounds[3]),xlabel='东向 / m',ylabel='北向 / m')
    left.ticklabel_format(useOffset=False,style='plain');left.set_title('任务边界附近的局部修正',loc='left',pad=16,fontsize=12,color=ink)
    for x,color,style in [(first,green,'-'),(second,purple,'--')]:
        profile=x['local_patch']['profile'];times=set(np.linspace(0,profile['duration_s'],900))
        for segment in profile['segments']:
            times.update([segment['t0_s'],segment['t1_s'],segment['t0_s']+segment['ramp_duration_s'],segment['t1_s']-segment['ramp_duration_s']])
        times=np.asarray(sorted(times));speed=[profile_state(profile,t)['speed_mps']for t in times]
        right.plot(times,speed,color=color,ls=style,lw=1.6)
    right.axhline(cfg['cruise_mps'],color=gray,ls=':',lw=.8)
    right.set(xlim=(0,math.ceil(max(x['local_patch']['profile']['duration_s']for x in results)/10)*10+20),
              ylim=(0,cfg['cruise_mps']*1.1),xlabel='局部运动时间 / s',ylabel='速度 / (m/s)')
    right.set_title('曲率变化处停稳再起步的速度律',loc='left',pad=16,fontsize=12,color=ink)
    fig.suptitle('A4 · 转弯空间核查与局部速度实验',fontsize=19,color=ink,y=.97)
    fig.text(.5,.914,f'固定 A3 扫描端点与导航域：最小半径 {cfg["nominal_uuv_minimum_radius_m"]:g} m 时，{nominal["certified_infeasible_endpoint_count"]} / {nominal["mandatory_scan_pose_endpoints_checked"]} 个必要端点空间不足',ha='center',fontsize=11,color='#586b7d')
    fig.legend(handles=[Patch(facecolor='#e2eff4',edgecolor='#718998',label='固定导航域'),
                        Line2D([],[],color=gray,ls='--',label='原扫描线'),Line2D([],[],color=green,label='局部修正'),
                        Line2D([],[],color=red,ls=':',label='原端点转弯越界')],loc='upper center',bbox_to_anchor=(.29,.25),ncol=2,frameon=False,fontsize=9)
    fig.legend(handles=[Line2D([],[],color=green,label=f'R = {patch["turn_radius_m"]:g} m'),Line2D([],[],color=purple,ls='--',label=f'R = {second["local_patch"]["turn_radius_m"]:g} m'),
                        Line2D([],[],color=gray,ls=':',label=f'巡航上限 {cfg["cruise_mps"]:g} m/s')],loc='upper center',bbox_to_anchor=(.76,.25),ncol=2,frameon=False,fontsize=9)
    fig.text(.5,.15,f'局部试验：100 m 扫描 + 掉头 + 100 m 扫描；R20 用时 {patch["profile"]["duration_s"]:.2f} s，R40 用时 {second["local_patch"]["profile"]["duration_s"]:.2f} s',ha='center',fontsize=10,color=ink)
    coverage=first['counterfactual_coverage']
    fig.text(.5,.103,f'仅缩进两个端点且掉头不扫描：覆盖 {coverage["after_one_patch_coverage_ratio"]*100:.4f}%；原 A3 为 {coverage["baseline_recomputed_covered_area_m2"]/coverage["target_area_m2"]*100:.4f}%。',ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.061,f'切向加速度上限 {cfg["tangential_accel_mps2"]:g} m/s²，横向加速度上限 {cfg["lateral_accel_mps2"]:g} m/s²；参数为工程示例，完整任务尚未通过运动验收。',ha='center',fontsize=9,color='#586b7d')
    fig.text(.5,.024,'左图为真实任务边界的局部视窗；区域外不等于陆地。下一步须把导航水域与覆盖目标分开设计。',ha='center',fontsize=9,color='#586b7d')
    fig.canvas.draw()
    if args.qa_script_dir:
        sys.path.insert(0,str(args.qa_script_dir))
        from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig,json_out=args.output_dir/'alignment.json',require_panel_labels=True,strict=True)
    stem=args.output_dir/'motion_feasibility_a4'
    fig.savefig(stem.with_suffix('.png'),dpi=300);fig.savefig(stem.with_suffix('.pdf'));fig.savefig(stem.with_suffix('.svg'))
    plt.close(fig)
    (args.output_dir/'figure_manifest.json').write_text(json.dumps({'source_result_sha256':[hashlib.sha256((p/'result.json').read_bytes()).hexdigest()for p in (args.run_dir,args.comparison_run_dir)],
        'plot_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'output_sha256':{ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest()for ext in ('png','pdf','svg')},
        'local_display_bounds_m':bounds,'display_is_local_only':True,'full_mission_feasibility_claimed':False},indent=2)+'\n')
    print(stem.with_suffix('.png'))


if __name__=='__main__':main()
