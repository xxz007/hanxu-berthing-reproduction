"""Build the common UUV clock and scan windows directly from certified curves."""
import hashlib
import json
from pathlib import Path

from ..curved_timing import CurvedClock, continuous_distance_bounds
from .control import canonical


def run(*, routes_dir, scans_dir, route_audit, scan_audit, config, output_dir):
    cfg = json.loads(config.read_text())
    sourcefile, repairsfile = routes_dir/'result.json', scans_dir/'result.json'
    source, repairs = (json.loads(p.read_text()) for p in (sourcefile, repairsfile))
    route_check, scan_check = (json.loads(p.read_text()) for p in (route_audit, scan_audit))
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    if canonical({k: v for k, v in source.items() if k != 'total_computation_seconds'}) != cfg['source_a6_scientific_sha256']:
        raise ValueError('Joint curve science differs from the fixed reproduction case')
    if canonical({k: v for k, v in repairs.items() if k != 'source_result_sha256'}) != cfg['source_repair_scientific_sha256']:
        raise ValueError('Repair schedule differs from the fixed reproduction case')
    if (not route_check['passed'] or not scan_check['passed']
            or route_check['source_result_sha256'] != digest(sourcefile)
            or scan_check['route_source_result_sha256'] != digest(sourcefile)
            or scan_check['repair_source_result_sha256'] != digest(repairsfile)):
        raise ValueError('Matching passed route and scan audits required')
    output_dir.mkdir(parents=True, exist_ok=False)
    settings = dict(tolerance_m=cfg['distance_tolerance_m'], numerical_margin_m=cfg['numerical_margin_m'], max_refinements=cfg['max_distance_refinements'])
    first_body, second_body = (source['platforms'][n] for n in ('uuv_1', 'uuv_2'))
    first = CurvedClock(first_body['pieces'], first_body['profile'])
    clocks = None
    for delay in cfg['uuv2_departure_candidates_s']:
        second = CurvedClock(second_body['pieces'], second_body['profile'], delay)
        end = max(first.finish_s, second.finish_s)
        pair = continuous_distance_bounds(first, second, end, **settings)
        if pair['converged'] and pair['minimum_distance_lower_m'] >= cfg['minimum_separation_m']:
            clocks = {'uuv_1': first, 'uuv_2': second}
            break
    if clocks is None:
        raise ValueError('No certified UUV departure pair in the finite candidate set')
    platforms = {}
    for name, clock in clocks.items():
        body = source['platforms'][name]
        holds = []
        if clock.departure_s > 0:
            holds.append(dict(t0_s=0., t1_s=clock.departure_s, pose=list(clock.initial_pose), kind='predeparture_hold', scanning=False))
        if clock.finish_s < end:
            holds.append(dict(t0_s=clock.finish_s, t1_s=end, pose=list(clock.final_pose), kind='completed_hold', scanning=False))
        platforms[name] = dict(pieces=body['pieces'], profile=body['profile'], length_m=body['length_m'], departure_s=clock.departure_s, motion_finish_s=clock.finish_s, initial_pose=list(clock.initial_pose), final_pose=list(clock.final_pose), holds=holds, source_task_ids=body['task_ids'])
    surface = source['platforms']['usv']
    platforms['usv'] = {'initial_pose': list(CurvedClock(surface['pieces'], surface['profile']).initial_pose)}
    windows = []
    for name in clocks:
        body = source['platforms'][name]
        offset = 0
        for index, leg in enumerate(body['legs']):
            count = len(leg['pieces'])
            if leg['scanning']:
                a = body['profile']['segments'][offset]['t0_s']
                b = body['profile']['segments'][offset+count-1]['t1_s']
                windows.append(dict(task_id=leg['task_id'], platform=name, kind='source_scan', source_leg_index=index, local_start_s=a, local_end_s=b, global_start_s=a+clocks[name].departure_s, global_end_s=b+clocks[name].departure_s))
            offset += count
    for repair in repairs['selected_repairs']:
        name = repair['platform']
        windows.append(dict(task_id=repair['task_id'], platform=name, kind='repair_scan', source_leg_index=repair['source_transit_leg_index'], local_start_s=repair['local_start_s'], local_end_s=repair['local_end_s'], global_start_s=repair['local_start_s']+clocks[name].departure_s, global_end_s=repair['local_end_s']+clocks[name].departure_s))
    windows.sort(key=lambda w: (w['platform'], w['global_start_s'], w['task_id']))
    result = dict(kind='joint_scan_schedule', mission_end_s=end, platforms=platforms, scan_windows=windows, coverage_lower_ratio_unchanged=repairs['coverage_lower_ratio'], distance_pairs={'uuv_1__uuv_2': {k: v for k, v in pair.items() if k != 'partition'} | {'certificate_file': 'certificates/uuv_pair.json'}})
    (output_dir/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    (output_dir/'uuv_pair.json').write_text(json.dumps(pair, indent=2)+'\n')
    (output_dir/'provenance.json').write_text(json.dumps({'input_sha256': {str(p.resolve()): digest(p) for p in (sourcefile, repairsfile, route_audit, scan_audit, config)}, 'result_sha256': digest(output_dir/'result.json'), 'pair_sha256': digest(output_dir/'uuv_pair.json')}, indent=2)+'\n')
