"""A9 independent schedule audit plus frozen-condition and cost comparison checks."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def comparison_errors(result, baseline, config, baseline_config):
    errors = []
    for key in ('source_a7_scientific_sha256', 'target_geometry_sha256',
                'hazards_geometry_sha256', 'navigation_sha256', 'mobile_design_range_m',
                'minimum_separation_m', 'hazard_margin_m', 'usv_radius_m',
                'usv_cruise_mps', 'tangential_accel_mps2', 'lateral_accel_mps2'):
        if config[key] != baseline_config[key]:
            errors.append('changed A8 constraint: ' + key)
    for key in ('uuv_platforms', 'scan_windows', 'mission_end_s', 'coverage_lower_ratio_unchanged'):
        if result[key] != baseline[key]:
            errors.append('changed A8 task: ' + key)
    if result['stage'] != 'A9' or result['mobile_usv']['initial_pose'] != baseline['mobile_usv']['initial_pose']:
        errors.append('stage or original USV pose')
    legs = result['mobile_usv']['legs']
    length = sum(p['profile']['length_m'] for p in legs)
    duration = sum(p['profile']['duration_s'] for p in legs)
    old = baseline['strategies']['scheduled_mobile']
    expected = {'source_a8_length_m': old['usv_length_m'],
                'source_a8_transfer_count': len(baseline['mobile_usv']['legs']),
                'source_a8_motion_duration_s': old['usv_motion_duration_s'],
                'optimized_length_m': length, 'optimized_transfer_count': len(legs),
                'length_saved_m': old['usv_length_m'] - length,
                'length_saved_ratio': 1 - length / old['usv_length_m'],
                'strict_length_improvement': length < old['usv_length_m'] - 1e-6,
                'all_constraints_certified': True}
    if result['comparison'] != expected:
        errors.append('comparison totals or improvement claim')
    strategy = result['strategies']['scheduled_mobile']
    if set(strategy['pairs']) != {'usv__' + name for name in baseline['uuv_platforms']}:
        errors.append('missing or extra whole mission distance pair')
    if abs(strategy['usv_length_m'] - length) > 1e-6 or abs(strategy['usv_motion_duration_s'] - duration) > 1e-6:
        errors.append('strategy cost totals')
    if not expected['strict_length_improvement']:
        errors.append('no strict length improvement over A8')
    for pair in strategy['pairs'].values():
        if (not pair['converged'] or pair['minimum_distance_lower_m'] < config['minimum_separation_m']
                or pair['maximum_distance_upper_m'] > config['mobile_design_range_m']):
            errors.append('whole mission A9 constraints')
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir', 'a8-run-dir', 'a8-audit-json', 'a7-run-dir', 'prepared-dir', 'navigation-dir', 'json-out'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    read = lambda p: json.loads(p.read_text())
    canon = lambda x: hashlib.sha256(json.dumps(x, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode()).hexdigest()
    result = read(args.run_dir / 'result.json'); cfg = read(args.run_dir / 'config.json')
    baseline = read(args.a8_run_dir / 'result.json'); audit = read(args.a8_audit_json)
    errors = comparison_errors(result, baseline, cfg, read(args.a8_run_dir / 'config.json'))
    if canon({k: v for k, v in baseline.items() if k != 'total_computation_seconds'}) != cfg['source_a8_scientific_sha256']:
        errors.append('frozen A8 scientific signature')
    if not audit['passed'] or audit['source_result_sha256'] != hashlib.sha256((args.a8_run_dir / 'result.json').read_bytes()).hexdigest():
        errors.append('matching passed A8 audit')
    schedule_out = args.json_out.with_name(args.json_out.stem + '_schedule.json')
    command = [sys.executable, str(ROOT / 'research_tests/audit_support_sensitivity.py')]
    for name in ('run_dir', 'a7_run_dir', 'prepared_dir', 'navigation_dir'):
        command.extend(['--' + name.replace('_', '-'), str(getattr(args, name))])
    command.extend(['--json-out', str(schedule_out)])
    process = subprocess.run(command, capture_output=True, text=True)
    schedule = read(schedule_out)
    if process.returncode or not schedule['passed']:
        errors.extend(schedule['errors'])
        errors.append('independent full schedule audit')
    report = {**schedule, 'passed': not errors, 'errors': errors,
              'stage': 'A9', 'comparison': result['comparison'],
              'frozen_A8_conditions_verified': not errors}
    args.json_out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'certificate_audits'}, indent=2))
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
