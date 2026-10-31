"""Coarse global planning and fine-chart local repair on saved GSHHG grids.

This is the position-planning experiment of thesis section 3.3, not a claim of
continuous curvature, vessel dynamics or author-identical Fig3.16 coordinates.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from berthing_planner.coordinates import Mercator
from berthing_planner.occupancy import OccupancyQuadTree
from berthing_planner.replanning import replan_path
from berthing_planner.search import astar


def chart_section(path, bounds):
    """Split one contiguous chart crossing without moving global endpoints.

A micrometre inset makes right/top crossings belong to the half-open fine map.
Only inserted chart anchors are inset; all original waypoints stay unchanged.
"""
    x0, y0, x1, y1 = bounds
    box = (x0+1e-6, y0+1e-6, x1-1e-6, y1-1e-6)
    spans = []
    for i, (a, b) in enumerate(zip(path, path[1:])):
        enter, leave = 0.0, 1.0
        for axis, lo, hi in ((0, box[0], box[2]), (1, box[1], box[3])):
            delta = b[axis]-a[axis]
            if delta == 0:
                if not lo <= a[axis] <= hi:
                    enter, leave = 1.0, 0.0
                    break
            else:
                lower, upper = sorted(((lo-a[axis])/delta, (hi-a[axis])/delta))
                enter, leave = max(enter, lower), min(leave, upper)
        if enter < leave:
            spans.append((i, enter, leave))
    if not spans:
        raise ValueError('Initial path does not enter the selected fine chart')
    for left, right in zip(spans, spans[1:]):
        if left[0]+1 != right[0] or left[2] != 1.0 or right[1] != 0.0:
            raise ValueError('This experiment requires one contiguous fine-chart crossing')
    first, t0, _ = spans[0]
    last, _, t1 = spans[-1]
    entry = tuple(path[first][k]+t0*(path[first+1][k]-path[first][k]) for k in (0, 1))
    exit_point = tuple(path[last][k]+t1*(path[last+1][k]-path[last][k]) for k in (0, 1))
    prefix = list(path[:first+1])
    if prefix[-1] != entry:
        prefix.append(entry)
    suffix = list(path[last+1:])
    if not suffix or suffix[0] != exit_point:
        suffix.insert(0, exit_point)
    local = [entry]+list(path[first+1:last+1])+[exit_point]
    local = [p for i, p in enumerate(local) if i == 0 or p != local[i-1]]
    return prefix, local, suffix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--allow-margin-relaxation', action='store_true',
                        help='Only after buffered-map failure, test land-only repair and disclose it')
    args = parser.parse_args()
    output = args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_cross_chart_r8')
    output.mkdir(parents=True, exist_ok=False)
    files = [args.map_dir/name for name in ('config.json', 'coarse_map.json', 'fine_map.json')]
    map_provenance = json.loads((args.map_dir/'provenance.json').read_text())
    for path in files:
        if hashlib.sha256(path.read_bytes()).hexdigest() != map_provenance['output_sha256'][path.name]:
            raise ValueError('Map bundle input hash changed: '+path.name)
    config, coarse, fine = [json.loads(p.read_text()) for p in files]
    begin = time.perf_counter()
    coarse_tree = OccupancyQuadTree(coarse['bounds'], coarse['rows_bottom_to_top'])
    initial = astar(coarse_tree, config['start_EN_m'], config['goal_EN_m'], parent_policy='theta_relaxation')
    prefix, local, suffix = chart_section(initial['path'], fine['bounds'])
    attempts = []
    result = None
    for use_margin in ((True, False) if args.allow_margin_relaxation else (True,)):
        field = 'rows_bottom_to_top' if use_margin else 'land_rows_bottom_to_top'
        fine_tree = OccupancyQuadTree(fine['bounds'], fine[field])
        attempt = {'margin_m': config['safety_margin_m'] if use_margin else 0.0,
                   'fine_leaves': len(fine_tree.leaves)}
        try:
            repair = replan_path(fine_tree, local, **config['local_replan'])
        except ValueError as error:
            attempt.update(status='no_valid_repair', reason=str(error))
            attempts.append(attempt)
            continue
        final = prefix[:-1]+repair['path']+suffix[1:]
        checks = {
            'coarse_initial_path_clear': all(coarse_tree.line_of_sight(a, b) for a, b in zip(initial['path'], initial['path'][1:])),
            'fine_initial_path_clear': all(fine_tree.line_of_sight(a, b) for a, b in zip(local, local[1:])),
            'fine_final_path_clear': all(fine_tree.line_of_sight(a, b) for a, b in zip(repair['path'], repair['path'][1:])),
            'global_start_unchanged': final[0] == tuple(config['start_EN_m']),
            'global_goal_unchanged': final[-1] == tuple(config['goal_EN_m']),
            'outside_prefix_preserved': final[:len(prefix)] == prefix,
            'outside_suffix_preserved': final[-len(suffix):] == suffix,
            'outside_prefix_clear_on_coarse_chart': all(coarse_tree.line_of_sight(a, b) for a, b in zip(prefix, prefix[1:])),
            'outside_suffix_clear_on_coarse_chart': all(coarse_tree.line_of_sight(a, b) for a, b in zip(suffix, suffix[1:])),
            'land_never_relaxed': all(fine_tree.line_of_sight(a, b) for a, b in zip(repair['path'], repair['path'][1:]))}
        # A changed route must demonstrate an actual fine-chart collision first.
        assert all(v for k, v in checks.items() if k != 'fine_initial_path_clear')
        assert repair['changed'] == (not checks['fine_initial_path_clear'])
        attempt.update(status='repair_passed', changed=repair['changed'])
        attempts.append(attempt)
        projection = Mercator(**config['projection'])
        result = {'status': 'independent_cross_chart_path_passed', 'initial': initial,
                  'coarse_chart_leaves': len(coarse_tree.leaves), 'fine_original_subpath': local,
                  'chart_entry_EN_m': local[0], 'chart_exit_EN_m': local[-1],
                  'outside_prefix': prefix, 'outside_suffix': suffix, 'local_repair': repair,
                  'final_path': final, 'final_lonlat_deg': [projection.inverse(*p) for p in final],
                  'length_m': sum(math.dist(a, b) for a, b in zip(final, final[1:])),
                  'checks': checks, 'margin_relaxed': not use_margin,
                  'active_margin_m': attempt['margin_m']}
        break
    if result is None:
        result = {'status': 'no_valid_cross_chart_repair', 'initial': initial,
                  'fine_original_subpath': local, 'outside_prefix': prefix, 'outside_suffix': suffix}
    result.update(attempts=attempts, config=config, total_seconds=time.perf_counter()-begin,
                  conditions={'endpoints': 'approximate figure reading, not author exact coordinates',
                              'chart_scales': 'c and f GSHHG with frozen engineering grids',
                              'output': 'position polyline, no curvature/heading/dynamic claim',
                              'author_figure_strictly_reproduced': False})
    (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    snapshot = output/'code_snapshot'; snapshot.mkdir()
    for path in (Path(__file__), *(ROOT/'src/berthing_planner'/name for name in (
            'coordinates.py', 'quadtree.py', 'occupancy.py', 'replanning.py', 'search.py'))):
        shutil.copy2(path, snapshot/path.name)
    (output/'provenance.json').write_text(json.dumps({
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'input_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in snapshot.iterdir()},
        'result_sha256': hashlib.sha256((output/'result.json').read_bytes()).hexdigest()}, indent=2)+'\n')
    print(json.dumps({k: result.get(k) for k in ('status', 'length_m', 'margin_relaxed', 'checks', 'attempts', 'total_seconds')}, ensure_ascii=False), flush=True)
    print(output)
    if result['status'] != 'independent_cross_chart_path_passed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
