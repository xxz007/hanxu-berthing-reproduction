"""Plot a saved, independently audited geometric coverage run on the actual A1 map."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from chart_geometry import Geos


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--audit-json',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path)
    args=parser.parse_args()
    run=args.run_dir
    result=json.loads((run/'result.json').read_text())
    provenance=json.loads((run/'provenance.json').read_text())
    audit=json.loads(args.audit_json.read_text())
    if result['status']!='geometric_coverage_passed' or not audit['passed']:
        raise ValueError('Only a passed, independently audited saved run may be displayed')
    if hashlib.sha256((run/'result.json').read_bytes()).hexdigest()!=provenance['result_sha256']:
        raise ValueError('Saved result changed')
    for filename,digest in provenance['geometry_sha256'].items():
        if hashlib.sha256((run/filename).read_bytes()).hexdigest()!=digest:
            raise ValueError('Saved coverage geometry changed')
    if hashlib.sha256((args.prepared_dir/'searchable.geojson').read_bytes()).hexdigest()!=result['target_geometry_sha256']:
        raise ValueError('Displayed target differs from planned target')
    a1_manifest=json.loads((args.prepared_dir/'manifest.json').read_text())
    for name in ('land','excluded_task','searchable','task_boundary'):
        if hashlib.sha256((args.prepared_dir/(name+'.geojson')).read_bytes()).hexdigest()!=a1_manifest['derived_files_sha256'][name+'.geojson']:
            raise ValueError('Displayed map geometry changed: '+name)
    args.output_dir.mkdir(parents=True,exist_ok=False)
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.path import Path as MplPath
    from matplotlib.patches import PathPatch,Patch
    from matplotlib.lines import Line2D

    plt.rcParams.update({'font.family':'sans-serif',
                         'font.sans-serif':['Noto Sans CJK SC','Arial','DejaVu Sans'],
                         'font.size':10,'axes.unicode_minus':False,
                         'pdf.fonttype':42,'svg.fonttype':'none',
                         'figure.facecolor':'white','savefig.facecolor':'white'})
    ink,land_color,excluded_color='#19334b','#b8b7ac','#f1e7dc'
    scan_color,transit_color,supplement_color='#078b8b','#b88330','#a94d76'
    covered_color,repeated_color,missed_color='#c3e0ec','#748cba','#d15167'

    def draw_geometry(ax,geometry,facecolor='none',edgecolor='none',linewidth=.4,zorder=2):
        kind=geometry['type']
        if kind=='GeometryCollection':
            for g in geometry['geometries']:
                draw_geometry(ax,g,facecolor,edgecolor,linewidth,zorder)
        elif kind=='MultiPolygon':
            for polygon in geometry['coordinates']:
                draw_geometry(ax,{'type':'Polygon','coordinates':polygon},facecolor,edgecolor,linewidth,zorder)
        elif kind=='Polygon':
            vertices,codes=[],[]
            for index,ring in enumerate(geometry['coordinates']):
                xy=np.asarray(ring,dtype=float)/1000
                if len(xy)<4:
                    continue
                signed=np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])
                if (signed>0)!=(index==0):
                    xy=xy[::-1]
                vertices.extend(xy)
                codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
            if vertices:
                ax.add_patch(PathPatch(MplPath(vertices,codes),facecolor=facecolor,edgecolor=edgecolor,
                                       lw=linewidth,zorder=zorder))

    shapes={name:json.loads((args.prepared_dir/(name+'.geojson')).read_text())['features'][0]['geometry']
            for name in ('land','excluded_task','searchable','task_boundary')}
    coverage_shapes={name:json.loads((run/(name+'.geojson')).read_text())['features'][0]['geometry']
                     for name in ('covered','repeated','missed')}
    # Clip only background drawing geometry to the display viewport. The full
    # target and quantitative coverage geometries must remain wholly visible.
    viewport={'type':'Polygon','coordinates':[[[500,700],[5250,700],[5250,4400],[500,4400],[500,700]]]}
    with Geos() as geos:
        if not geos.covers(viewport,shapes['task_boundary']):
            raise ValueError('Display viewport would hide part of the task')
        background_area_before=geos.measure(shapes['land'])['area_m2']
        shapes={name:geos.operation('intersection',geometry,viewport) for name,geometry in shapes.items()}
        background_area_after=geos.measure(shapes['land'])['area_m2']
    fig,axes=plt.subplots(1,2,figsize=(13.8,7.7))
    fig.subplots_adjust(left=.065,right=.985,bottom=.25,top=.78,wspace=.18)
    for ax,label,title in zip(axes,('a','b'),('覆盖航段与避障连接','覆盖、重复与剩余漏扫')):
        ax.set(xlim=(.50,5.25),ylim=(.70,4.40),aspect='equal',
               xlabel='局部东向 E（km）',ylabel='局部北向 N（km）')
        ax.set_facecolor('#f7fbfc')
        draw_geometry(ax,shapes['land'],land_color,'#8d918c',.4,zorder=2)
        draw_geometry(ax,shapes['excluded_task'],excluded_color,zorder=3)
        draw_geometry(ax,shapes['searchable'],'#edf5f6',zorder=3)
        draw_geometry(ax,shapes['task_boundary'],edgecolor='#d2b283',linewidth=.7,zorder=4)
        ax.set_title(title,fontsize=13,color=ink,pad=15)
        ax.annotate(label,xy=(0,1),xycoords='axes fraction',xytext=(-15,13),textcoords='offset points',
                    fontsize=12,fontweight='bold',color=ink)
        ax.spines[['top','right']].set_visible(False)
        ax.spines[['left','bottom']].set_color('#aab9c1')
        ax.grid(alpha=.15,lw=.5,zorder=0)
        ax.set_axisbelow(True)
    for leg in result['legs']:
        points=np.asarray(leg['points'])/1000
        color=scan_color if leg['kind']=='sweep' else supplement_color if leg['scanning'] else transit_color
        axes[0].plot(points[:,0],points[:,1],color=color,lw=1.10 if leg['scanning'] else .85,
                     ls='-' if leg['scanning'] else '--',zorder=6 if leg['scanning'] else 5)
    long_scans=[leg for leg in result['legs'] if leg['scanning'] and leg['length_m']>800]
    if long_scans:
        for index in np.linspace(0,len(long_scans)-1,min(5,len(long_scans)),dtype=int):
            a,b=np.asarray(long_scans[index]['points'])[[0,-1]]/1000
            centre=(a+b)/2
            direction=(b-a)/np.linalg.norm(b-a)*.10
            axes[0].annotate('',xy=centre+direction,xytext=centre-direction,
                             arrowprops={'arrowstyle':'-|>','color':scan_color,'lw':1.3},zorder=7)
    for point,marker,label,offset in ((result['path'][0],'o','起点',(8,-12)),
                                      (result['path'][-1],'*','终点',(6,8))):
        x,y=np.asarray(point)/1000
        axes[0].scatter(x,y,s=45 if marker=='o' else 65,marker=marker,color=ink,zorder=9)
        axes[0].annotate(label,xy=(x,y),xytext=offset,textcoords='offset points',fontsize=9,color=ink,zorder=10)
    draw_geometry(axes[1],coverage_shapes['covered'],covered_color,zorder=5)
    draw_geometry(axes[1],coverage_shapes['repeated'],repeated_color,zorder=6)
    draw_geometry(axes[1],coverage_shapes['missed'],missed_color,missed_color,.65,zorder=7)
    stats=result['vector_coverage']
    fig.suptitle('长滩近岸 · 单平台区域覆盖规划',fontsize=20,color=ink,y=.965)
    fig.text(.5,.905,'理想扫宽 100 m · 航线间距 80 m · 连接航段不计扫描',ha='center',fontsize=11,color='#586b7d')
    fig.legend(handles=[Line2D([],[],color=scan_color,lw=1.6,label='往复扫描'),
                        Line2D([],[],color=transit_color,lw=1.2,ls='--',label='避障连接（不扫描）'),
                        Line2D([],[],color=supplement_color,lw=1.6,label='补充扫描'),
                        Patch(facecolor=excluded_color,label='任务内排除区'),
                        Patch(facecolor=covered_color,label='已覆盖水域'),
                        Patch(facecolor=repeated_color,label='至少两次覆盖'),
                        Patch(facecolor=missed_color,label='剩余漏扫')],
               loc='upper center',bbox_to_anchor=(.5,.205),ncol=4,frameon=False,fontsize=10)
    fig.text(.5,.115,f'覆盖率 {100*stats["coverage_ratio"]:.3f}% · 重复覆盖率 {100*stats["repeated_ratio"]:.2f}% · 路径 {result["length_m"]/1000:.2f} km',
             ha='center',fontsize=11,color=ink)
    fig.text(.5,.073,f'固定候选水域 {stats["target_area_m2"]/1e6:.3f} km²；剩余漏扫 {stats["missed_area_m2"]:.0f} m²。红色描边标注遗漏位置，面积统计使用原始几何。',
             ha='center',fontsize=10,color='#586b7d')
    fig.text(.5,.035,'NOAA ENC 地图背景 · 理想圆形扫描足迹，仅评价位置折线与几何覆盖；曲率、速度、三维和跟踪控制尚未验证。',
             ha='center',fontsize=9,color='#586b7d')
    fig.canvas.draw()
    if args.qa_script_dir:
        sys.path.insert(0,str(args.qa_script_dir))
        from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig,json_out=args.output_dir/'alignment.json',require_panel_labels=True,strict=True)
    stem=args.output_dir/'open_chart_coverage_a2'
    fig.savefig(stem.with_suffix('.png'),dpi=300)
    fig.savefig(stem.with_suffix('.pdf'))
    fig.savefig(stem.with_suffix('.svg'))
    plt.close(fig)
    (args.output_dir/'figure_manifest.json').write_text(json.dumps({
        'result_sha256':provenance['result_sha256'],'audit_sha256':hashlib.sha256(args.audit_json.read_bytes()).hexdigest(),
        'plot_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'output_sha256':{ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest() for ext in ('png','pdf','svg')},
        'coverage_ratio':stats['coverage_ratio'],'repeated_ratio':stats['repeated_ratio'],'length_m':result['length_m'],
        'display_bounds_local_m':[500,700,5250,4400],
        'background_land_area_before_display_crop_m2':background_area_before,
        'background_land_area_after_display_crop_m2':background_area_after,
        'full_task_visible':True,'quantitative_geometry_cropped':False,
        'curvature_control_verified':False,'raw_data_uploaded':False},ensure_ascii=False,indent=2)+'\n')
    print(stem.with_suffix('.png'))


if __name__=='__main__':
    main()
