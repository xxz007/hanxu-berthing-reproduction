"""Separate engineered full-merge transit spine plus canonical RS audit.

Original R5/R6 and R7 branches remain unchanged. Failed stern smoothing is
not replaced by a prefix and is not represented as a success here.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from berthing_planner.curves import Piece, collision_free, curvature_maximum, wrap
from berthing_planner.raster_map import RasterQuadTree
from berthing_planner.stitching import piece_length, stitch_rs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--merge-dir', type=Path, required=True)
    parser.add_argument('--search-dir', type=Path, required=True)
    parser.add_argument('--map-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    output = args.output_dir or ROOT/'outputs'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_merged_transit_r5')
    output.mkdir(parents=True, exist_ok=False)
    summary, input_hashes = {}, {}
    merge_summary_file = args.merge_dir/'summary.json'
    merge_provenance = json.loads(merge_summary_file.read_text())
    input_hashes[str(merge_summary_file.resolve().relative_to(ROOT))] = hashlib.sha256(
        merge_summary_file.read_bytes()).hexdigest()
    for variant in ('tight', 'baseline', 'loose'):
        map_file = args.map_dir/(variant+'_map.json')
        search_file = args.search_dir/(variant+'_transit_goal_theta_relaxation.json')
        merge_file = args.merge_dir/(variant+'_transit_goal.json')
        for f in (map_file, search_file, merge_file):
            input_hashes[str(f.resolve().relative_to(ROOT))] = hashlib.sha256(f.read_bytes()).hexdigest()
        for f in (map_file, search_file):
            if input_hashes[str(f.resolve().relative_to(ROOT))] != merge_provenance['input_sha256'][
                    str(f.resolve().relative_to(ROOT))]:
                raise ValueError('Map/search input differs from the engineering merge audit')
        grid = json.loads(map_file.read_text())
        tree = RasterQuadTree(grid)
        raw = json.loads(search_file.read_text())
        merged = json.loads(merge_file.read_text())
        if merged['status'] != 'engineering_geometry_passed' or merged['original_path'] != raw['path']:
            raise ValueError('Expected a complete independently checked merged transit spine')
        spine = [Piece(**p) for p in merged['pieces']]
        if not collision_free(spine, tree.obstacles, tree.bounds):
            raise AssertionError('Full input spine is not clear on the selected map')
        start, goal = (*raw['path'][0], 0.0), (*raw['path'][-1], 0.0)
        result = stitch_rs(spine, start, goal, .005, tree.obstacles, tree.bounds,
                           raw['length_m'], reverse_weight=2.0, anchor_step=100.0)
        result.update(variant=variant, goal_name='transit_goal',
                      branch='engineering_geometric_merge_then_canonical_rs',
                      author_equivalence=False, source_algorithm_gate_passed=False,
                      start_pose=start, goal_pose=goal, reverse_weight=2.0,
                      anchor_step_m=100.0, maximum_allowed_curvature_per_m=.005,
                      full_spine_geometry_collision_free=True,
                      spine_length_m=sum(map(piece_length, spine)),
                      original_search_path=raw['path'], original_point_count=len(raw['path']),
                      merged_virtual_path=merged['merged_virtual_path'], merge_history=merged['merges'],
                      terminal_connector='canonical_only', no_los_shortcut=True,
                      no_stern_case_or_synthetic_prefix_substitution=True)
        if result['status'] == 'retained_route_passed':
            pieces = result['pieces']
            first, last = pieces[0].evaluate(0), pieces[-1].evaluate(1)
            joins = [(a.evaluate(1), b.evaluate(0)) for a, b in zip(pieces, pieces[1:])]
            check = {
                'collision_free': collision_free(pieces, tree.obstacles, tree.bounds),
                'start_position_error_m': math.dist(first[:2], start[:2]),
                'goal_position_error_m': math.dist(last[:2], goal[:2]),
                'start_heading_error_rad': abs(wrap(first[2]-start[2])),
                'goal_heading_error_rad': abs(wrap(last[2]-goal[2])),
                'maximum_join_position_error_m': max((math.dist(a[:2], b[:2]) for a, b in joins), default=0),
                'maximum_join_heading_error_rad': max((abs(wrap(a[2]-b[2])) for a, b in joins), default=0),
                'maximum_actual_curvature_per_m': max(map(curvature_maximum, pieces)),
                'maximum_curvature_jump_per_m': max((abs(a[3]-b[3]) for a, b in joins), default=0),
                'integrated_length_m': sum(map(piece_length, pieces)),
                'integrated_reverse_m': sum(piece_length(p) for p in pieces if p.direction < 0)}
            assert check['collision_free']
            assert max(check['start_position_error_m'], check['goal_position_error_m'],
                       check['maximum_join_position_error_m']) < 1e-5
            assert max(check['start_heading_error_rad'], check['goal_heading_error_rad'],
                       check['maximum_join_heading_error_rad']) < 1e-7
            assert check['maximum_actual_curvature_per_m'] <= .005+1e-9
            assert abs(check['integrated_length_m']-result['length_m']) < 1e-7
            assert abs(check['integrated_reverse_m']-result['reverse_m']) < 1e-7
            result['independent_recheck'] = check
        result['spine_pieces'] = [asdict(p) for p in spine]
        result['pieces'] = [asdict(p) for p in result['pieces']]
        shutil.copy2(map_file, output/map_file.name)
        (output/(variant+'_transit_goal.json')).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        summary[variant] = {k: v for k, v in result.items()
                            if k not in ('pieces', 'spine_pieces', 'trace', 'original_search_path')}
        print(variant, result['status'], result.get('length_m'), flush=True)
    files = [Path(__file__), *sorted((ROOT/'src/berthing_planner').glob('*.py')),
             ROOT/'docs/g2_merge_sources.json']
    code_hashes = {}
    for source in files:
        name = str(source.relative_to(ROOT))
        target = output/'code_snapshot'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        code_hashes[name] = hashlib.sha256(source.read_bytes()).hexdigest()
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    (output/'provenance.json').write_text(json.dumps({
        'created_utc': datetime.now(timezone.utc).isoformat(), 'python': platform.python_version(),
        'input_sha256': input_hashes, 'code_sha256': code_hashes,
        'conditions': {'start_and_goal_EN_yaw_rad': 0, 'maximum_curvature_per_m': .005,
                       'reverse_weight': 2, 'anchor_step_m': 100,
                       'terminal_activation_origin': '2/3 of the original complete A-star polyline length',
                       'initial_anchor_selection': 'Existing stitch_rs first-third cost-ranked engineering choice',
                       'terminal_candidates': 'Existing canonical RS only; no stern-axis extension'},
        'author_equivalence': False, 'source_algorithm_gate_passed': False,
        'scope': 'New transit-only branch; original R5/R6/R7 untouched; no hull/dynamics/sea-trial validation'
    }, ensure_ascii=False, indent=2)+'\n')
    print(output, flush=True)


if __name__ == '__main__':
    main()
