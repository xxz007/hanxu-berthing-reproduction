"""Plot an independently audited A3 allocation and continuous-clock geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from chart_geometry import Geos


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir', type=Path, required=True)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--audit-json', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--qa-script-dir', type=Path)
    args = parser.parse_args()
    result = json.loads((args.run_dir/'result.json').read_text())
    provenance = json.loads((args.run_dir/'provenance.json').read_text())
    audit = json.loads(args.audit_json.read_text())
    if result['status'] != 'cooperative_geometry_passed' or not audit['passed']:
        raise ValueError('Figure requires a passed, independently audited A3 run')
    if hashlib.sha256((args.run_dir/'result.json').read_bytes()).hexdigest() != provenance['result_sha256']:
        raise ValueError('Saved result changed')
    if hashlib.sha256((args.prepared_dir/'searchable.geojson').read_bytes()).hexdigest() != result['target_geometry_sha256']:
        raise ValueError('Figure target differs from planned target')
    manifest = json.loads((args.prepared_dir/'manifest.json').read_text())
    shapes = {}
    viewport = {'type': 'Polygon', 'coordinates': [[[500,700],[5250,700],[5250,4400],[500,4400],[500,700]]]}
    with Geos() as geos:
        for name in ('land', 'excluded_task', 'searchable', 'task_boundary'):
            file = args.prepared_dir/(name+'.geojson')
            if hashlib.sha256(file.read_bytes()).hexdigest() != manifest['derived_files_sha256'][file.name]:
                raise ValueError('Changed map drawing input: '+name)
            geometry = json.loads(file.read_text())['features'][0]['geometry']
            if name == 'task_boundary' and not geos.covers(viewport, geometry):
                raise ValueError('Display crop hides part of the task boundary')
            shapes[name] = geos.operation('intersection', geometry, viewport)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.path import Path as MplPath
    from matplotlib.patches import PathPatch

    plt.rcParams.update({'font.family': 'sans-serif',
                         'font.sans-serif': ['Noto Sans CJK SC', 'Arial', 'DejaVu Sans'],
                         'font.size': 10, 'axes.unicode_minus': False,
                         'pdf.fonttype': 42, 'svg.fonttype': 'none',
                         'figure.facecolor': 'white', 'savefig.facecolor': 'white'})
    ink = '#19334b'
    colors = {'uuv_1': '#078b8b', 'uuv_2': '#b36b16', 'usv': '#744c99'}

    def draw_geometry(ax, geometry, facecolor, edgecolor='none', linewidth=.5):
        kind = geometry['type']
        if kind == 'GeometryCollection':
            for shape in geometry['geometries']:
                draw_geometry(ax, shape, facecolor, edgecolor, linewidth)
        elif kind == 'MultiPolygon':
            for polygon in geometry['coordinates']:
                draw_geometry(ax, {'type': 'Polygon', 'coordinates': polygon}, facecolor, edgecolor, linewidth)
        elif kind == 'Polygon':
            vertices, codes = [], []
            for i, ring in enumerate(geometry['coordinates']):
                xy = np.asarray(ring, dtype=float)/1000
                signed = np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])
                if (signed > 0) != (i == 0):
                    xy = xy[::-1]
                vertices.extend(xy)
                codes.extend([MplPath.MOVETO]+[MplPath.LINETO]*(len(xy)-2)+[MplPath.CLOSEPOLY])
            if vertices:
                ax.add_patch(PathPatch(MplPath(vertices, codes), facecolor=facecolor,
                                       edgecolor=edgecolor, lw=linewidth))

    fig, axes = plt.subplots(1, 2, figsize=(13.8, 7.8))
    fig.subplots_adjust(left=.07, right=.97, bottom=.27, top=.81, wspace=.23)
    for index, ax in enumerate(axes):
        ax.set_box_aspect(3.7/4.75)
        ax.spines[['top', 'right']].set_visible(False)
        for spine in ax.spines.values():
            spine.set_color('#b8c6ce')
        ax.tick_params(colors='#4b6070', labelsize=10)
        ax.text(-.04, 1.065, chr(97+index), transform=ax.transAxes,
                fontsize=12, fontweight='bold', color=ink, va='bottom')
        ax.grid(color='#e5eaed', linewidth=.5, alpha=.7)
    map_ax, time_ax = axes
    draw_geometry(map_ax, shapes['land'], '#b8b7ac')
    draw_geometry(map_ax, shapes['excluded_task'], '#f1e7dc')
    draw_geometry(map_ax, shapes['searchable'], '#e5f2f1')
    draw_geometry(map_ax, shapes['task_boundary'], 'none', '#657b8c', .7)
    for key in ('uuv_1', 'uuv_2', 'usv'):
        platform = result['platforms'][key]
        for leg in platform['legs']:
            points = np.asarray(leg['points'])/1000
            map_ax.plot(points[:,0], points[:,1], color=colors[key],
                        ls='-' if leg['scanning'] or key == 'usv' else '--',
                        lw=1.4 if key == 'usv' else .8, alpha=.9 if key == 'usv' else .8)
        for point, marker in ((platform['path'][0], 'o'), (platform['path'][-1], '*')):
            map_ax.scatter(point[0]/1000, point[1]/1000, s=35 if marker == 'o' else 60,
                           marker=marker, color=colors[key], edgecolors='white', linewidths=.7, zorder=8)
    map_ax.set(xlim=(.5,5.25), ylim=(.7,4.4), xlabel='东向 / km', ylabel='北向 / km')
    map_ax.set_title('任务分配与避障航线', loc='left', pad=16, fontsize=12, color=ink)
    end = result['mission_end_s']
    times = set(np.linspace(0, end, 1500))
    for platform in result['platforms'].values():
        for segment in platform['timeline']['segments']:
            times.update((segment['t0_s'], segment['t1_s']))
    for record in result['pairwise_distances'].values():
        times.update((record['minimum_time_s'], record['maximum_time_s']))
    times = np.asarray(sorted(times))
    positions = {}
    for key, platform in result['platforms'].items():
        segments = platform['timeline']['segments']
        ends = np.asarray([s['t1_s'] for s in segments])
        selected = np.clip(np.searchsorted(ends, times, side='left'), 0, len(segments)-1)
        p0 = np.asarray([segments[i]['p0'] for i in selected])
        p1 = np.asarray([segments[i]['p1'] for i in selected])
        start = np.asarray([segments[i]['t0_s'] for i in selected])
        finish = np.asarray([segments[i]['t1_s'] for i in selected])
        fraction = np.clip((times-start)/(finish-start), 0, 1)
        positions[key] = p0+(p1-p0)*fraction[:,None]
    curve_specs = [('usv', 'uuv_1', colors['uuv_1'], 'USV—UUV 1'),
                   ('usv', 'uuv_2', colors['uuv_2'], 'USV—UUV 2'),
                   ('uuv_1', 'uuv_2', '#566678', 'UUV 1—UUV 2')]
    for a, b, color, label in curve_specs:
        distance = np.linalg.norm(positions[a]-positions[b], axis=1)
        time_ax.plot(times/3600, distance, color=color, lw=1.05, label=label)
    time_ax.axhline(25, color='#566678', lw=.8, ls=':')
    time_ax.axhline(3000, color='#b04e58', lw=.8, ls='--')
    time_ax.set(xlim=(0,end/3600), ylim=(10,6000), yscale='log',
                xlabel='任务时间 / h', ylabel='水平距离 / m（对数刻度）')
    time_ax.set_yticks([10,25,100,500,1000,3000], ['10','25','100','500','1000','3000'])
    time_ax.minorticks_off()
    time_ax.set_title('完整时钟的间距与支援核查', loc='left', pad=16, fontsize=12, color=ink)
    fig.suptitle('长滩近岸 · 1 艘 USV + 2 艘 UUV 协同覆盖', fontsize=19, color=ink, y=.97)
    fig.text(.5, .914, '固定 86 条扫描任务 · 水面支援不计扫描 · 等待与结束停留纳入全程核查',
             ha='center', fontsize=11, color='#586b7d')
    map_handles = [Line2D([],[],color=colors[u],lw=1.5,
                         label=('UUV '+u[-1]+' 扫描 / 连接') if u != 'usv' else 'USV 支援航线')
                   for u in ('uuv_1','uuv_2','usv')]
    fig.legend(handles=map_handles, loc='upper center', bbox_to_anchor=(.29,.208),
               ncol=3, frameon=False, fontsize=9)
    fig.legend(handles=[Line2D([],[],color=color,lw=1.2,label=label) for _,_,color,label in curve_specs],
               loc='upper center', bbox_to_anchor=(.76,.208), ncol=3, frameon=False, fontsize=9)
    min_distance = min(v['minimum_distance_m'] for v in result['pairwise_distances'].values())
    max_support = max(result['pairwise_distances']['usv__'+u]['maximum_distance_m'] for u in ('uuv_1','uuv_2'))
    fig.text(.5,.157, f'任务时长 {end/3600:.2f} h · 全平台最小水平间距 {min_distance:.1f} m · 最大支援距离 {max_support:.1f} m',
             ha='center', fontsize=11, color=ink)
    fig.text(.5,.118, f'覆盖 {result["coverage"]["coverage_ratio"]*100:.3f}%；剩余漏扫 {result["coverage"]["missed_area_m2"]:.0f} m²。距离下限 25 m；支援上限 3000 m 仅用于 USV—UUV。',
             ha='center', fontsize=10, color='#586b7d')
    fig.text(.5,.073, '航速设定：UUV 1.5 m/s、USV 3 m/s。三平台已部署后开始计时，不含回收；双 UUV 使用两个覆盖平台。',
             ha='center', fontsize=9, color='#586b7d')
    fig.text(.5,.033, 'NOAA ENC 真实地图背景 · 二维点平台几何回放；转弯、动力学、深度与实际声学通信尚未验证。',
             ha='center', fontsize=9, color='#586b7d')
    fig.canvas.draw()
    if args.qa_script_dir:
        sys.path.insert(0, str(args.qa_script_dir))
        from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig, json_out=args.output_dir/'alignment.json',
                                            require_panel_labels=True, strict=True)
    stem = args.output_dir/'cooperative_coverage_a3'
    fig.savefig(stem.with_suffix('.png'), dpi=300)
    fig.savefig(stem.with_suffix('.pdf'))
    fig.savefig(stem.with_suffix('.svg'))
    plt.close(fig)
    (args.output_dir/'figure_manifest.json').write_text(json.dumps({
        'result_sha256': provenance['result_sha256'],
        'audit_sha256': hashlib.sha256(args.audit_json.read_bytes()).hexdigest(),
        'plot_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'output_sha256': {ext:hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest() for ext in ('png','pdf','svg')},
        'display_bounds_local_m':[500,700,5250,4400], 'full_target_visible':True,
        'quantitative_geometry_cropped':False,
        'time_plot_samples':len(times), 'exact_extrema_source':'analytically audited result; samples are display only',
        'minimum_horizontal_distance_m':min_distance,'maximum_support_range_m':max_support,
        'three_dimensional_or_motion_feasibility_verified':False}, ensure_ascii=False, indent=2)+'\n')
    print(stem.with_suffix('.png'))


if __name__ == '__main__':
    main()
