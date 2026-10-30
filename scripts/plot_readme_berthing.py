"""Plot saved berthing routes over their exact reconstructed obstacle grid.

Uses existing research plotting dependencies; never runs a planner or overwrites
an output directory. Raw map data and the source PDF are not exported.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from berthing_planner.curves import Piece, collision_free
from berthing_planner.raster_map import RasterQuadTree


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--combination-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--qa-script-dir', type=Path, help='Optional local panel-alignment auditor')
    args = parser.parse_args()
    files = [args.combination_dir / name for name in (
        'baseline_map.json',
        'baseline_transit_goal_journal_E0_canonical.json',
        'baseline_stern_goal_journal_E0_canonical_plus_stern_axis.json')]
    grid, transit, stern = [json.loads(p.read_text()) for p in files]
    tree = RasterQuadTree(grid)
    routes = [transit, stern]
    pieces = [[Piece(**p) for p in r['pieces']] for r in routes]
    if not all(r['status'] == 'retained_route_passed' for r in routes):
        raise ValueError('Only previously validated saved routes may be displayed')
    if not all(collision_free(p, tree.obstacles, tree.bounds) for p in pieces):
        raise ValueError('Saved routes do not pass the displayed obstacle geometry')
    args.output_dir.mkdir(parents=True, exist_ok=False)

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PatchCollection
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch, Rectangle

    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ['Noto Sans CJK SC', 'Arial', 'DejaVu Sans'], 'font.size': 11,
        'axes.unicode_minus': False, 'pdf.fonttype': 42, 'svg.fonttype': 'none',
        'figure.facecolor': 'white', 'savefig.facecolor': 'white'})
    forward, reverse, ink, blocked = '#00858c', '#c74469', '#19334b', '#778797'
    fig, axes = plt.subplots(1, 2, figsize=(14, 7.2))
    fig.subplots_adjust(left=.065, right=.985, bottom=.19, top=.75, wspace=.18)
    x0, y0, x1, y1 = tree.bounds
    dx, dy = (x1 - x0) / grid['n'], (y1 - y0) / grid['n']
    blocked_cells = [(i, j) for j, row in enumerate(grid['rows_bottom_to_top'])
                     for i, value in enumerate(row) if value]
    for ax, r, route, label, title in zip(
            axes, routes, pieces, ('a', 'b'), ('通航到达', '船艉倒车抵泊')):
        rectangles = [Rectangle((x0 + i*dx, y0 + j*dy), dx, dy) for i, j in blocked_cells]
        ax.add_collection(PatchCollection(rectangles, facecolor=blocked, edgecolor='none', zorder=2))
        for piece in route:
            samples = piece.samples(4)
            ax.plot([s[0] for s in samples], [s[1] for s in samples], lw=2.4,
                    color=reverse if piece.direction < 0 else forward,
                    linestyle='--' if piece.direction < 0 else '-', zorder=4)
        # Motion arrows follow sample order, including the reverse segment.
        for direction, color in ((1, forward), (-1, reverse)):
            candidates = [p for p in route if p.direction == direction]
            if candidates:
                piece = max(candidates, key=lambda p: p.length)
                before, after = piece.evaluate(.45), piece.evaluate(.55)
                ax.annotate('', xy=after[:2], xytext=before[:2], zorder=5,
                            arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': 2,
                                        'mutation_scale': 13})
        for name, marker in (('start_pose', 'o'), ('goal_pose', '*')):
            x, y, yaw = r[name]
            ax.scatter(x, y, marker=marker, s=65 if marker == '*' else 38,
                       color=ink, zorder=7)
            ax.annotate('', xy=(x + 270*math.cos(yaw), y + 270*math.sin(yaw)),
                        xytext=(x, y), zorder=6,
                        arrowprops={'arrowstyle': '-|>', 'color': ink, 'lw': 1.6,
                                    'mutation_scale': 12})
        for point in r['cusps']:
            ax.scatter(*point, s=68, facecolors='white', edgecolors=reverse,
                       linewidth=1.8, zorder=7)
        ax.set(xlim=(x0, x1), ylim=(y0, y1), aspect='equal',
               xlabel='图上 E 坐标（m）', ylabel='图上 N 坐标（m）')
        ax.set_title(f'{title}\n路径 {r["length_m"]:.1f} m · 倒车 {r["reverse_m"]:.1f} m',
                     fontsize=13, color=ink, pad=12)
        ax.annotate(label, xy=(0, 1), xycoords='axes fraction', xytext=(-16, 12),
                    textcoords='offset points', fontsize=12, fontweight='bold', color=ink)
        ax.grid(alpha=.13, zorder=0)
        ax.set_axisbelow(True)
        ax.spines[['top', 'right']].set_visible(False)
        ax.spines[['left', 'bottom']].set_color('#b6c2cc')

    fig.suptitle('恢复港口图实验 · 障碍环境中的靠泊路径', fontsize=20, y=.965, color=ink)
    fig.legend(handles=[
        Patch(facecolor=blocked, label='实验阻塞栅格'),
        Line2D([], [], color=forward, lw=2.4, label='前进'),
        Line2D([], [], color=reverse, lw=2.4, linestyle='--', label='倒车'),
        Line2D([], [], color=ink, marker='>', lw=1.4, label='船头朝向'),
        Line2D([], [], color=ink, marker='o', lw=0, label='起点'),
        Line2D([], [], color=ink, marker='*', markersize=9, lw=0, label='终点'),
        Line2D([], [], color=reverse, marker='o', markerfacecolor='white', lw=0, label='换向点')],
        loc='upper center', bbox_to_anchor=(.5, .90), ncol=4, frameon=False, fontsize=11)
    fig.text(.5, .105, '灰色为实际用于规划与碰撞检查的阻塞栅格；彩色箭头表示沿路径的运动方向。',
             ha='center', fontsize=11, color=ink)
    fig.text(.5, .062, '障碍由论文图像独立恢复；只检查中心线几何，不代表船体包络或动力学验证。',
             ha='center', fontsize=10, color='#586b7d')

    fig.canvas.draw()
    boxes = [ax.get_window_extent().bounds for ax in axes]
    assert math.isclose(boxes[0][2], boxes[1][2], abs_tol=.1)
    assert math.isclose(boxes[0][1], boxes[1][1], abs_tol=.1)
    assert math.isclose(boxes[0][3], boxes[1][3], abs_tol=.1)
    (args.output_dir / 'panel_geometry.json').write_text(json.dumps(boxes, indent=2) + '\n')
    if args.qa_script_dir:
        sys.path.insert(0, str(args.qa_script_dir))
        from audit_panel_alignment import require_matplotlib_panel_alignment
        require_matplotlib_panel_alignment(fig, json_out=args.output_dir / 'alignment.json',
                                          require_panel_labels=True, strict=True)
    stem = args.output_dir / 'berthing_results_with_obstacles'
    fig.savefig(stem.with_suffix('.png'), dpi=300)
    fig.savefig(stem.with_suffix('.pdf'))
    fig.savefig(stem.with_suffix('.svg'))
    plt.close(fig)
    manifest = {
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'kind': 'Saved routes redrawn over the exact saved obstacle grid; no planner rerun',
        'contains_reconstructed_obstacle_geometry': True,
        'contains_paper_page_or_private_notes': False,
        'obstacle_representation': 'Blocked cells only; polygon interiors are not added',
        'obstacle_cells_drawn_per_panel': len(blocked_cells),
        'map_bounds_EN_m': tree.bounds, 'grid_n': grid['n'], 'display_sampling_m': 4,
        'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'input_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'output_sha256': {stem.with_suffix('.' + ext).name:
                          hashlib.sha256(stem.with_suffix('.' + ext).read_bytes()).hexdigest()
                          for ext in ('png', 'pdf', 'svg')},
        'matplotlib': matplotlib.__version__,
        'saved_routes_collision_rechecked': True,
        'route_metrics': {name: {k: r[k] for k in (
            'length_m', 'reverse_m', 'maximum_curvature_per_m', 'cusps', 'start_pose', 'goal_pose')}
            for name, r in zip(('transit', 'stern'), routes)}}
    (args.output_dir / 'berthing_results_with_obstacles_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(stem.with_suffix('.png'))


if __name__ == '__main__':
    main()
