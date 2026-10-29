"""R4: source-formula and recovered-map audit, preserving unsuccessful cases."""
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
from berthing_planner.curves import g2_smooth, curvature_maximum, collision_free, wrap
from berthing_planner.raster_map import RasterQuadTree


def corner_budget(path, maximum_curvature):
    """Independent evaluation of paper eq. (15) and shared-segment budgets.

    Segment i must accommodate d at both of its endpoint corners. This audit
    neither deletes nodes nor substitutes an overlapping pair of spirals.
    """
    c1, c2 = 7.2364, 2*(math.sqrt(6)-1)/5
    c3 = (c2+4)/(c1+6)
    trims = [0.0]*len(path)
    bisected_trims = [0.0]*len(path)
    corners = []
    for i in range(1, len(path)-1):
        a, b, c = path[i-1:i+2]
        left, right = math.dist(a, b), math.dist(b, c)
        dot = sum((b[j]-a[j])*(c[j]-b[j]) for j in range(2))/(left*right)
        gamma = math.acos(max(-1, min(1, dot)))
        beta = gamma/2
        d = (c2+4)**2/(54*c3)*math.sin(beta)/(maximum_curvature*math.cos(beta)**2)
        # Literal eqs. (4),(5),(11), before the implementation's midpoint join.
        gap = abs(2*d*math.cos(beta)*(1-c3*(1+c2+6/(c2+4))))
        trims[i] = d
        # 2008 conference reference, eq. (21). Length audit only: this is not
        # an implemented bisection-curve generator or a multi-corner merge.
        bisected_d = (1+1/math.cos(beta))*1.1228*math.sin(beta/2)/(maximum_curvature*math.cos(beta/2)**2)
        bisected_trims[i] = bisected_d
        corners.append({'node_index':i, 'gamma_rad':gamma, 'beta_rad':beta,
                        'd_m':d, 'literal_printed_c1_join_gap_m':gap,
                        'reference_2008_single_bisection_d_m':bisected_d})
    segments = [{'segment_index':i, 'length_m':math.dist(a,b),
                 'required_trim_m':trims[i]+trims[i+1],
                 'remaining_m':math.dist(a,b)-trims[i]-trims[i+1],
                 'reference_2008_single_bisection_remaining_m':math.dist(a,b)-bisected_trims[i]-bisected_trims[i+1]}
                for i,(a,b) in enumerate(zip(path,path[1:]))]
    return {'corners':corners, 'segments':segments,
            'overlap_segments':[s['segment_index'] for s in segments if s['remaining_m'] < -1e-8],
            'reference_2008_single_bisection_overlap_segments':[s['segment_index'] for s in segments
                if s['reference_2008_single_bisection_remaining_m'] < -1e-8]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--search-dir', type=Path, required=True)
    parser.add_argument('--map-dir', type=Path, required=True)
    parser.add_argument('--source-pdf', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    local = ROOT/'docs/source_local.json'
    source = args.source_pdf or Path(json.loads(local.read_text())['g2cbs_pdf'])
    manifest = json.loads((ROOT/'docs/g2cbs_source_manifest.json').read_text())
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != manifest['sha256']:
        raise ValueError('G2CBS PDF differs from the inspected source')
    output = args.output_dir or ROOT/'outputs'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'_g2cbs_r4_audit')
    output.mkdir(parents=True, exist_ok=False)
    records, inputs = {}, {}
    for variant in ('tight','baseline','loose'):
        map_path = args.map_dir/(variant+'_map.json')
        tree = RasterQuadTree(json.loads(map_path.read_text()))
        inputs[str(map_path.resolve())] = hashlib.sha256(map_path.read_bytes()).hexdigest()
        for goal in ('transit_goal','stern_goal'):
            path_file = args.search_dir/f'{variant}_{goal}_theta_relaxation.json'
            inputs[str(path_file.resolve())] = hashlib.sha256(path_file.read_bytes()).hexdigest()
            result = json.loads(path_file.read_text())
            for field in ('path','separate_los_shortcut'):
                path = result[field]
                record = {'variant':variant, 'goal':goal, 'input_kind':field,
                          'path':path, 'maximum_curvature_per_m':0.005,
                          'budget':corner_budget(path, 0.005),
                          'postprocessing_applied':field=='separate_los_shortcut'}
                if record['budget']['overlap_segments']:
                    record['status'] = 'insufficient_segment_budget'
                    # Ensure production code rejects the independently detected overlap.
                    try:
                        g2_smooth(path, 0.005)
                    except ValueError as error:
                        if 'overlap' not in str(error) and 'exceeds' not in str(error):
                            raise
                        record['production_rejection'] = str(error)
                    else:
                        raise AssertionError('Overlapping corners were silently accepted')
                else:
                    pieces = g2_smooth(path, 0.005)
                    record['pieces'] = [asdict(p) for p in pieces]
                    record['maximum_actual_curvature_per_m'] = max(map(curvature_maximum,pieces),default=0)
                    joins = [(a.evaluate(1),b.evaluate(0)) for a,b in zip(pieces,pieces[1:])]
                    record['maximum_join_errors'] = {
                        'position_m':max((math.dist(a[:2],b[:2]) for a,b in joins),default=0),
                        'heading_rad':max((abs(wrap(a[2]-b[2])) for a,b in joins),default=0),
                        'curvature_per_m':max((abs(a[3]-b[3]) for a,b in joins),default=0)}
                    record['blocked_piece_indices'] = [i for i,p in enumerate(pieces)
                        if not collision_free([p],tree.obstacles,tree.bounds)]
                    record['endpoint_error_m'] = max(math.dist(pieces[0].evaluate(0)[:2],path[0]),
                                                      math.dist(pieces[-1].evaluate(1)[:2],path[-1]))
                    record['status'] = 'curve_collides' if record['blocked_piece_indices'] else 'curve_geometry_passed'
                    if record['maximum_actual_curvature_per_m'] > 0.005+1e-10:
                        raise AssertionError('Actual curvature exceeds the configured bound')
                records[f'{variant}_{goal}_{field}'] = record
    (output/'records.json').write_text(json.dumps(records,indent=2)+'\n')
    summary = {k:{p:v for p,v in r.items() if p not in ('pieces','path','budget')}
               for k,r in records.items()}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    shutil.copy2(args.map_dir/'baseline_map.json',output/'baseline_map.json')
    hashes = {}
    for p in [Path(__file__), ROOT/'src/berthing_planner/curves.py',
              ROOT/'src/berthing_planner/raster_map.py',ROOT/'src/berthing_planner/quadtree.py',
              ROOT/'docs/g2cbs_source_manifest.json',ROOT/'docs/g2cbs_2008_source_manifest.json']:
        t = output/'code_snapshot'/p.relative_to(ROOT)
        t.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(p,t)
        hashes[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    (output/'provenance.json').write_text(json.dumps({
        'created_utc':datetime.now(timezone.utc).isoformat(), 'python':platform.python_version(),
        'g2cbs_source_sha256':digest, 'input_sha256':inputs, 'code_sha256':hashes,
        'curvature_source':'Han thesis printed p.38: minimum radius 200 m',
        'secondary_formula_source':'Yang/Sukkarieh ACRA 2008, eq.21; length-only audit, no bisection curve generation',
        'scope':'centerline geometry on independently recovered maps; no heading or RS integration',
        'R4_full_gate_passed':False,
        'remaining_gaps':['explicit short-turn merge rule','collision repair rule',
                          'author equivalence of path/map input'],
        'source_pdf_copied_or_uploaded':False},indent=2)+'\n')
    print(output)
    for k,r in records.items():
        print(k, r['status'], 'overlaps',r['budget']['overlap_segments'])


if __name__ == '__main__':
    main()
