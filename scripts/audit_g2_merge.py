"""Audit a labelled engineering short-turn merge without replacing source branches."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from berthing_planner.g2_merge import merge_adjacent_turns, subdivision_budget
from berthing_planner.raster_map import RasterQuadTree
from berthing_planner.stitching import piece_length


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--search-dir', type=Path, required=True)
    parser.add_argument('--map-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    output = args.output_dir or ROOT/'outputs'/(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_g2_merge_audit')
    output.mkdir(parents=True, exist_ok=False)
    source_hashes, records, summary = {}, {}, {}
    for variant in ('tight', 'baseline', 'loose'):
        map_path = args.map_dir/(variant+'_map.json')
        grid = json.loads(map_path.read_text())
        tree = RasterQuadTree(grid)
        source_hashes[str(map_path.resolve().relative_to(ROOT))] = hashlib.sha256(map_path.read_bytes()).hexdigest()
        shutil.copy2(map_path, output/map_path.name)
        for goal in ('transit_goal', 'stern_goal'):
            source = args.search_dir/f'{variant}_{goal}_theta_relaxation.json'
            source_hashes[str(source.resolve().relative_to(ROOT))] = hashlib.sha256(source.read_bytes()).hexdigest()
            original = json.loads(source.read_text())['path']
            audit = {str(n): subdivision_budget(original, .005, n)
                     for n in (1, 2, 4, 8, 16, 64, None)}
            result = merge_adjacent_turns(original, .005, tree.obstacles, tree.bounds)
            key = variant+'_'+goal
            if result['pieces']:
                result['length_m'] = sum(piece_length(p) for p in result['pieces'])
            result.update(variant=variant, goal_name=goal, original_point_count=len(original),
                          maximum_allowed_curvature_per_m=.005,
                          subdivision_budget_audit=audit,
                          endpoint_heading_scope='Original polyline tangent headings, not prescribed berthing poses',
                          source_algorithm_gate_passed=False,
                          no_los_shortcut=True, no_rs_replacement=True)
            result['pieces'] = [asdict(p) for p in result['pieces']]
            if 'rejected_pieces' in result:
                result['rejected_pieces'] = [asdict(p) for p in result['rejected_pieces']]
            (output/(key+'.json')).write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
            records[key] = result
            summary[key] = {k: result[k] for k in ('status', 'author_equivalence', 'original_point_count',
                                                 'merged_virtual_path', 'merges')}
            if 'checks' in result:
                summary[key].update(length_m=result['length_m'], checks=result['checks'])
    tracked = ('src/berthing_planner/g2_merge.py', 'src/berthing_planner/g2_bisection.py',
               'src/berthing_planner/curves.py', 'src/berthing_planner/stitching.py',
               'src/berthing_planner/raster_map.py', 'scripts/audit_g2_merge.py',
               'tests/test_g2_merge.py', 'docs/g2_merge_sources.json')
    code_hashes = {}
    for name in tracked:
        source = ROOT/name
        target = output/'code_snapshot'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        code_hashes[name] = hashlib.sha256(source.read_bytes()).hexdigest()
    record = {'kind': 'Independent engineering branch, not recovered author merging code',
              'source_algorithm_gate_passed': False, 'python': platform.python_version(),
              'input_sha256': source_hashes, 'code_sha256': code_hashes,
              'cases': summary, 'success_count': sum(r['status']=='engineering_geometry_passed'
                                                   for r in records.values())}
    (output/'summary.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    print(output)
    print(json.dumps({k: {'status': v['status'], 'merge_count': len(v['merges']),
                         'length_m': v.get('length_m')} for k, v in summary.items()}, ensure_ascii=False))


if __name__ == '__main__':
    main()
