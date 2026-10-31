"""Draw actual saved NOAA vector features and a separately specified task boundary."""
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--qa-script-dir', type=Path)
    args = parser.parse_args()
    p = args.prepared_dir
    manifest = json.loads((p/'manifest.json').read_text())
    for rel, digest in manifest['derived_files_sha256'].items():
        if hashlib.sha256((p/rel).read_bytes()).hexdigest() != digest:
            raise ValueError('Prepared geometry hash changed: '+rel)
    if hashlib.sha256((p/'summary.json').read_bytes()).hexdigest() != manifest['summary_sha256']:
        raise ValueError('Prepared summary hash changed')
    cfg = json.loads((p/'config.json').read_text())
    summary = json.loads((p/'summary.json').read_text())
    args.output_dir.mkdir(parents=True, exist_ok=False)
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.path import Path as MplPath
    from matplotlib.patches import PathPatch, Patch
    from matplotlib.lines import Line2D
    from matplotlib.colors import ListedColormap, BoundaryNorm

    plt.rcParams.update({'font.family':'sans-serif',
                         'font.sans-serif':['Noto Sans CJK SC','Arial','DejaVu Sans'],
                         'font.size':10,'axes.unicode_minus':False,
                         'pdf.fonttype':42,'svg.fonttype':'none',
                         'figure.facecolor':'white','savefig.facecolor':'white'})
    ink, land_color, candidate, task_color = '#19334b','#b8b7ac','#148b83','#c07b13'
    hazard_color, excluded_color = '#c6cbd0','#f0e3d6'
    depth_cmap = ListedColormap(['#edf5f6','#d6ebef','#afd2e0','#79b0c8','#4687a6'])
    depth_norm = BoundaryNorm([-100,0,5,10,20,10000],depth_cmap.N)

    def draw_geometry(ax, geometry, facecolor='none', edgecolor='none', linewidth=.5, alpha=1, zorder=2, linestyle='-'):
        kind = geometry['type']
        if kind == 'GeometryCollection':
            for g in geometry['geometries']:
                draw_geometry(ax,g,facecolor,edgecolor,linewidth,alpha,zorder,linestyle)
        elif kind == 'MultiPolygon':
            for polygon in geometry['coordinates']:
                draw_geometry(ax,{'type':'Polygon','coordinates':polygon},facecolor,edgecolor,linewidth,alpha,zorder,linestyle)
        elif kind == 'Polygon':
            vertices, codes = [], []
            for index, ring in enumerate(geometry['coordinates']):
                xy = np.asarray(ring,dtype=float)/1000
                if len(xy)<4:
                    continue
                signed = np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])
                if (signed>0) != (index==0):
                    xy = xy[::-1]
                vertices.extend(xy)
                codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
            if vertices:
                ax.add_patch(PathPatch(MplPath(vertices,codes),facecolor=facecolor,edgecolor=edgecolor,
                                       linewidth=linewidth,alpha=alpha,zorder=zorder,linestyle=linestyle))
        elif kind in ('LineString','MultiLineString'):
            lines = [geometry['coordinates']] if kind=='LineString' else geometry['coordinates']
            for line in lines:
                xy = np.asarray(line)/1000
                ax.plot(xy[:,0],xy[:,1],color=edgecolor,lw=linewidth,alpha=alpha,zorder=zorder,linestyle=linestyle)
        elif kind in ('Point','MultiPoint'):
            points = [geometry['coordinates']] if kind=='Point' else geometry['coordinates']
            if points:
                xy = np.asarray(points)/1000
                ax.scatter(xy[:,0],xy[:,1],s=20,marker='x',color=edgecolor,linewidths=.8,alpha=alpha,zorder=zorder)

    features = {name:json.loads((p/'features'/(name+'.geojson')).read_text())['features']
                for name in summary['feature_counts']}
    shapes = {name:json.loads((p/(name+'.geojson')).read_text())['features'][0]['geometry']
              for name in ('land','hazard_buffer','restriction_context','task_boundary','searchable','excluded_task')}
    fig, axes = plt.subplots(1,2,figsize=(13.4,8.2))
    fig.subplots_adjust(left=.065,right=.98,bottom=.255,top=.79,wspace=.20)
    x0,y0,x1,y1 = summary['bbox_local_m']
    for ax,label,title in zip(axes,('a','b'),('海图要素：岸线、岛屿与水深分区','任务设定：不规则边界与候选规划水域')):
        ax.set(xlim=(x0/1000,x1/1000),ylim=(y0/1000,y1/1000),aspect='equal',
               xlabel='局部东向 E（km）',ylabel='局部北向 N（km）')
        ax.set_facecolor('#f7fbfc')
        ax.set_title(title,fontsize=12,color=ink,pad=16)
        ax.annotate(label,xy=(0,1),xycoords='axes fraction',xytext=(-14,14),textcoords='offset points',
                    fontsize=12,fontweight='bold',color=ink)
        ax.spines[['top','right']].set_visible(False)
        ax.spines[['left','bottom']].set_color('#aab9c1')
        ax.grid(color='#b3c6ce',alpha=.20,lw=.5,zorder=0)
        ax.set_axisbelow(True)
    for f in features['DEPARE']:
        lower = f['properties'].get('DRVAL1')
        color = depth_cmap(depth_norm(lower)) if lower is not None else '#eeeeee'
        draw_geometry(axes[0],f['geometry'],facecolor=color)
    for name in ('CBLSUB','PIPSOL'):
        for f in features.get(name,[]):
            draw_geometry(axes[0],f['geometry'],edgecolor='#778a98',linewidth=.55,linestyle='--',zorder=3)
    for ax in axes:
        draw_geometry(ax,shapes['land'],facecolor=land_color,edgecolor='#888d87',linewidth=.35,zorder=4)
    for name in ('COALNE','SLCONS'):
        for f in features[name]:
            draw_geometry(axes[0],f['geometry'],facecolor=land_color,edgecolor='#5f6b73',linewidth=.5,zorder=5)
    for name in ('OBSTRN','UWTROC','WRECKS'):
        for f in features[name]:
            draw_geometry(axes[0],f['geometry'],facecolor='#a95865',edgecolor='#a95865',linewidth=.6,zorder=6)
    draw_geometry(axes[1],shapes['hazard_buffer'],facecolor=hazard_color,alpha=.9,zorder=3)
    draw_geometry(axes[1],shapes['excluded_task'],facecolor=excluded_color,alpha=.95,zorder=5)
    draw_geometry(axes[1],shapes['searchable'],facecolor=candidate,alpha=.80,zorder=6)
    for ax in axes:
        draw_geometry(ax,shapes['restriction_context'],edgecolor='#8d8494',linewidth=.8,linestyle=':',zorder=7)
        draw_geometry(ax,shapes['task_boundary'],edgecolor=task_color,linewidth=1.9,linestyle='--',zorder=8)
    fig.suptitle('长滩近岸 · 真实海图与不规则任务区域',fontsize=20,color=ink,y=.965)
    fig.text(.5,.91,'NOAA ENC：US5LGBCE / US5LGBDE · 1:12,000 · 地图准备阶段',ha='center',fontsize=11,color='#586b7d')
    fig.legend(handles=[Patch(facecolor=land_color,label='陆地／岛屿'),
                        Patch(facecolor='#afd2e0',label='海图水深分区'),
                        Line2D([],[],marker='x',color='#a95865',lw=0,label='礁石／沉船等障碍'),
                        Line2D([],[],color=task_color,lw=1.9,ls='--',label='自行设定的任务边界'),
                        Patch(facecolor=candidate,label='几何候选规划水域'),
                        Patch(facecolor=excluded_color,label='任务内排除区')],
               loc='upper center',bbox_to_anchor=(.5,.205),ncol=3,frameon=False,fontsize=10)
    fig.text(.5,.115,f'任务范围 {summary["task_area_m2"]/1e6:.3f} km²；几何候选水域 {summary["searchable_area_m2"]/1e6:.3f} km²。',
             ha='center',fontsize=11,color=ink)
    fig.text(.5,.075,'筛选参数：水深分区下界 ≥ 5 m；实体障碍至少外扩 50 m。紫灰点线保留海图管制区边界。',
             ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.035,'基于实际 S-57 矢量要素重绘；任务范围为仿真设定。管制区条件尚未作为实际作业许可判定；本阶段未生成覆盖航线。',
             ha='center',fontsize=9,color='#586b7d')
    fig.canvas.draw()
    if args.qa_script_dir:
        sys.path.insert(0,str(args.qa_script_dir))
        from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig,json_out=args.output_dir/'alignment.json',
                                          require_panel_labels=True,strict=True)
    stem = args.output_dir/'open_chart_stage_a'
    fig.savefig(stem.with_suffix('.png'),dpi=300)
    fig.savefig(stem.with_suffix('.pdf'))
    fig.savefig(stem.with_suffix('.svg'))
    plt.close(fig)
    (args.output_dir/'figure_manifest.json').write_text(json.dumps({
        'prepared_manifest_sha256':hashlib.sha256((p/'manifest.json').read_bytes()).hexdigest(),
        'plot_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'output_sha256':{ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest() for ext in ('png','pdf','svg')},
        'task_area_m2':summary['task_area_m2'],'candidate_area_m2':summary['searchable_area_m2'],
        'coverage_planner_run':False,'raw_charts_uploaded':False},ensure_ascii=False,indent=2)+'\n')
    print(stem.with_suffix('.png'))


if __name__ == '__main__':
    main()
