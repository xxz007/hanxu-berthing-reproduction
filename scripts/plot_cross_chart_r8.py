"""Draw the saved real-data R8 global route and its fine-chart local repair."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path, required=True)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--qa-script-dir', type=Path)
    args = parser.parse_args()
    files = [args.run_dir/'result.json', args.map_dir/'coarse_map.json', args.map_dir/'fine_map.json']
    run_provenance = json.loads((args.run_dir/'provenance.json').read_text())
    if hashlib.sha256(files[0].read_bytes()).hexdigest() != run_provenance['result_sha256']:
        raise ValueError('Saved result hash changed')
    for path in files[1:]:
        if hashlib.sha256(path.read_bytes()).hexdigest() != run_provenance['input_sha256'][path.name]:
            raise ValueError('Figure map differs from the map used by planning')
    result, coarse, fine = [json.loads(p.read_text()) for p in files]
    if result['status'] != 'independent_cross_chart_path_passed':
        raise ValueError('No validated cross-chart result to present')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, Rectangle

    plt.rcParams.update({'font.family': 'sans-serif',
                         'font.sans-serif': ['Noto Sans CJK SC', 'Arial', 'DejaVu Sans'],
                         'font.size': 11, 'axes.unicode_minus': False,
                         'pdf.fonttype': 42, 'svg.fonttype': 'none'})
    land_color, margin_color = '#8fa873', '#bac1c8'
    initial_color, final_color, ink = '#4477aa', '#c74469', '#19334b'
    ratios = [(m['bounds'][2]-m['bounds'][0])/(m['bounds'][3]-m['bounds'][1]) for m in (coarse, fine)]
    fig, axes = plt.subplots(1, 2, figsize=(14, 7.8), gridspec_kw={'width_ratios': ratios})
    fig.subplots_adjust(left=.065, right=.985, bottom=.20, top=.76, wspace=.18)
    for ax, grid, label, title in zip(axes, (coarse, fine), ('a', 'b'), ('全域：粗图规划与局部更新', '局部细图：新增障碍与避障修正')):
        land = np.asarray(grid['land_rows_bottom_to_top'], dtype=bool)
        buffered = np.asarray(grid['rows_bottom_to_top'], dtype=bool)
        background = np.where(land, 2, np.where(buffered, 1, 0))
        x0, y0, x1, y1 = grid['bounds']
        extent = [(x0-11000000)/1000, (x1-11000000)/1000, y0/1000, y1/1000]
        ax.imshow(background, origin='lower', extent=extent, interpolation='nearest',
                  cmap=ListedColormap(['white', margin_color, land_color]), vmin=0, vmax=2,
                  aspect='equal', zorder=1)
        final_path = result['final_path'] if label == 'a' else result['local_repair']['path']
        initial_path = result['initial']['path'] if label == 'a' else result['fine_original_subpath']
        for path, color, style, width, order in (
                (final_path, final_color, '-', 2.5, 3),
                (initial_path, initial_color, '--', 1.7, 4)):
            ax.plot([(p[0]-11000000)/1000 for p in path], [p[1]/1000 for p in path],
                    color=color, linestyle=style, lw=width, zorder=order)
        if label == 'a':
            a, b, c, d = fine['bounds']
            ax.add_patch(Rectangle(((a-11000000)/1000, b/1000), (c-a)/1000, (d-b)/1000,
                                   fill=False, edgecolor=ink, linestyle=':', lw=1.3, zorder=5))
            for point, marker in ((result['final_path'][0], 'o'), (result['final_path'][-1], '*')):
                ax.scatter((point[0]-11000000)/1000, point[1]/1000, marker=marker,
                           color=ink, s=45, zorder=6)
        else:
            for point in (result['local_repair']['replacement_start'], result['local_repair']['replacement_end']):
                ax.scatter((point[0]-11000000)/1000, point[1]/1000, s=45,
                           facecolor='white', edgecolor=ink, linewidth=1.5, zorder=6)
        ax.set(xlim=extent[:2], ylim=extent[2:], xlabel='(E − 11,000,000 m) / km', ylabel='N / km')
        ax.set_title(title, fontsize=13, color=ink, pad=14)
        ax.annotate(label, xy=(0, 1), xycoords='axes fraction', xytext=(-17, 13),
                    textcoords='offset points', fontsize=12, fontweight='bold', color=ink)
        ax.spines[['top', 'right']].set_visible(False)
        ax.spines[['left', 'bottom']].set_color('#b6c2cc')
    fig.suptitle('新加坡海峡 · GSHHG 跨图局部重规划', fontsize=20, y=.97, color=ink)
    fig.legend(handles=[Patch(facecolor=land_color, label='陆地'),
                        Patch(facecolor=margin_color, label='50 m 裕度的保守栅格外扩'),
                        Line2D([], [], color=initial_color, linestyle='--', lw=1.7, label='粗图初次路径'),
                        Line2D([], [], color=final_color, lw=2.5, label='细图局部修正路径'),
                        Line2D([], [], color=ink, marker='o', markerfacecolor='white', lw=0, label='局部替换锚点')],
               loc='upper center', bbox_to_anchor=(.5, .91), ncol=3, frameon=False, fontsize=11)
    fig.text(.5, .115, f'全程 {result["initial"]["length_m"]/1000:.3f} → {result["length_m"]/1000:.3f} km；'
             f'仅替换原路线中 {result["local_repair"]["replaced_original_length_m"]/1000:.3f} km 航段，图外路径及起终点保持。',
             ha='center', fontsize=11, color=ink)
    fig.text(.5, .07, 'GSHHG 2.3.7 · Wessel & Smith；细格约 97.66 × 63.48 m，裕度未放宽。',
             ha='center', fontsize=10, color='#586b7d')
    fig.text(.5, .033, '独立重建：起终点为图 3.16 近似读数；本图只展示位置路径，不代表作者原始地图或曲率／动力学验收。',
             ha='center', fontsize=10, color='#586b7d')
    fig.canvas.draw()
    boxes = [a.get_window_extent().bounds for a in axes]
    assert math.isclose(boxes[0][1], boxes[1][1], abs_tol=.1)
    assert math.isclose(boxes[0][3], boxes[1][3], abs_tol=.1)
    (args.output_dir/'panel_geometry.json').write_text(json.dumps(boxes, indent=2)+'\n')
    if args.qa_script_dir:
        sys.path.insert(0, str(args.qa_script_dir))
        from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig, json_out=args.output_dir/'alignment.json',
                                          require_panel_labels=True, strict=True)
    stem = args.output_dir/'cross_chart_r8'
    fig.savefig(stem.with_suffix('.png'), dpi=300)
    fig.savefig(stem.with_suffix('.pdf'))
    fig.savefig(stem.with_suffix('.svg'))
    plt.close(fig)
    (args.output_dir/'cross_chart_r8_manifest.json').write_text(json.dumps({
        'kind': 'Saved GSHHG cross-chart run; no planner rerun by plotting',
        'input_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'output_sha256': {stem.with_suffix('.'+ext).name: hashlib.sha256(stem.with_suffix('.'+ext).read_bytes()).hexdigest()
                          for ext in ('png', 'pdf', 'svg')},
        'raw_data_uploaded': False, 'author_exact_figure': False,
        'initial_length_m': result['initial']['length_m'], 'final_length_m': result['length_m'],
        'active_margin_m': result['active_margin_m']}, indent=2)+'\n')
    print(stem.with_suffix('.png'))


if __name__ == '__main__':
    main()
