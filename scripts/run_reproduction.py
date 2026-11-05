"""A10: reproduce the fixed ENC A1–A9 experiment in a fresh output directory.

Uses the existing environment; installs nothing. Raw chart snapshots must match
the public pins. All intermediate results are rebuilt under this run directory.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_archives(directory):
    charts = json.loads((ROOT / 'configs/open_chart_long_beach_stage_a.json').read_text())['charts']
    records = {}
    for chart in charts:
        path = directory / (chart['cell'] + '.zip')
        if not path.is_file() or path.stat().st_size != chart['bytes'] or digest(path) != chart['sha256']:
            raise ValueError('Pinned chart snapshot missing or changed: ' + chart['cell'])
        records[path.name] = {'sha256': digest(path), 'bytes': path.stat().st_size, 'url': chart['url']}
    return records


def commands(out):
    """Explicit dependency order; every generated input belongs to this new run."""
    a1, grid, a2, a3 = (out / p for p in ('a1_map', 'grid', 'a2', 'a3'))
    nav, a5, a6, repair, a7, a8, a9 = (out / p for p in ('a5_nav', 'a5_adapted', 'a6', 'a6_repair', 'a7', 'a8', 'a9'))
    mapargs = ['--prepared-dir', a1]
    navargs = [*mapargs, '--navigation-dir', nav]
    return [
        ('a1_prepare', 'scripts/prepare_open_chart_stage_a.py', ['--archives-dir', out/'archives', '--output-dir', a1]),
        ('a1_audit', 'research_tests/audit_open_chart_stage_a.py', [*mapargs, '--json-out', out/'a1_audit.json']),
        ('a1_plot', 'scripts/plot_open_chart_stage_a.py', [*mapargs, '--output-dir', out/'a1_figure']),
        ('a2_grid', 'scripts/prepare_coverage_grid.py', [*mapargs, '--n', '2048', '--output-dir', grid]),
        ('a2_run', 'scripts/run_open_chart_coverage.py', [*mapargs, '--grid-dir', grid, '--output-dir', a2]),
        ('a2_audit', 'research_tests/audit_open_chart_coverage.py', [*mapargs, '--grid-dir', grid, '--run-dir', a2, '--json-out', out/'a2_audit.json']),
        ('a2_plot', 'scripts/plot_open_chart_coverage.py', [*mapargs, '--run-dir', a2, '--audit-json', out/'a2_audit.json', '--output-dir', out/'a2_figure']),
        ('a3_freeze', 'scripts/freeze_equivalent_a2_inputs.py', ['--a2-run-dir', a2, '--output-config', out/'a3_config.json']),
        ('a3_run', 'scripts/run_cooperative_coverage.py', [*mapargs, '--a2-run-dir', a2, '--grid-dir', grid, '--config', out/'a3_config.json', '--output-dir', a3]),
        ('a3_audit', 'research_tests/audit_cooperative_coverage.py', [*mapargs, '--run-dir', a3, '--a2-run-dir', a2, '--grid-dir', grid, '--json-out', out/'a3_audit.json']),
        ('a3_plot', 'scripts/plot_cooperative_coverage.py', [*mapargs, '--run-dir', a3, '--audit-json', out/'a3_audit.json', '--output-dir', out/'a3_figure']),
        ('a3_replay', 'scripts/build_cooperative_replay.py', [*mapargs, '--run-dir', a3, '--audit-json', out/'a3_audit.json', '--output', out/'a3_replay.html']),
        ('a4_study', 'scripts/run_motion_sensitivity.py', [*mapargs, '--a3-run-dir', a3, '--grid-dir', grid, '--output-dir', out/'a4_study']),
        ('a5_prepare', 'scripts/prepare_navigation_domain.py', [*mapargs, '--output-dir', nav]),
        ('a5_fixed', 'scripts/run_headland_connectors.py', [*navargs, '--a3-run-dir', a3, '--output-dir', out/'a5_fixed']),
        ('a5_obstructions', 'scripts/classify_connector_obstructions.py', ['--trial-dir', out/'a5_fixed', '--navigation-dir', nav, '--json-out', out/'a5_obstructions.json']),
        ('a5_adapted', 'scripts/run_adapted_headland_routes.py', [*navargs, '--a3-run-dir', a3, '--output-dir', a5]),
        ('a5_audit', 'research_tests/audit_headland_navigation.py', [*navargs, '--trial-dir', out/'a5_fixed', '--adapted-dir', a5, '--obstructions-json', out/'a5_obstructions.json', '--json-out', out/'a5_audit.json']),
        ('a6_run', 'scripts/run_joint_coverage.py', [*navargs, '--a3-run-dir', a3, '--a5-run-dir', a5, '--output-dir', a6]),
        ('a6_audit', 'research_tests/audit_joint_coverage.py', [*navargs, '--run-dir', a6, '--a3-run-dir', a3, '--a5-run-dir', a5, '--json-out', out/'a6_audit.json']),
        ('a6_repair', 'scripts/schedule_repair_scans.py', [*mapargs, '--run-dir', a6, '--audit-json', out/'a6_audit.json', '--output-dir', repair]),
        ('a6_repair_audit', 'research_tests/audit_repair_scans.py', [*mapargs, '--run-dir', a6, '--repair-dir', repair, '--route-audit-json', out/'a6_audit.json', '--json-out', out/'a6_repair_audit.json']),
        ('a6_plot', 'scripts/plot_joint_coverage.py', [*navargs, '--run-dir', a6, '--repair-dir', repair, '--audit-json', out/'a6_audit.json', '--repair-audit-json', out/'a6_repair_audit.json', '--output-dir', out/'a6_figure']),
        ('a7_run', 'scripts/run_curved_cooperation.py', [*navargs, '--a6-run-dir', a6, '--repair-dir', repair, '--route-audit-json', out/'a6_audit.json', '--repair-audit-json', out/'a6_repair_audit.json', '--output-dir', a7]),
        ('a7_audit', 'research_tests/audit_curved_cooperation.py', [*navargs, '--run-dir', a7, '--a6-run-dir', a6, '--repair-dir', repair, '--json-out', out/'a7_audit.json']),
        ('a7_plot', 'scripts/plot_curved_cooperation.py', [*navargs, '--run-dir', a7, '--a6-run-dir', a6, '--audit-json', out/'a7_audit.json', '--output-dir', out/'a7_figure']),
        ('a7_replay', 'scripts/build_curved_cooperation_replay.py', [*navargs, '--run-dir', a7, '--audit-json', out/'a7_audit.json', '--output', out/'a7_replay.html']),
        ('a8_run', 'scripts/run_support_sensitivity.py', [*navargs, '--a7-run-dir', a7, '--a7-audit-json', out/'a7_audit.json', '--output-dir', a8]),
        ('a8_audit', 'research_tests/audit_support_sensitivity.py', [*navargs, '--run-dir', a8, '--a7-run-dir', a7, '--json-out', out/'a8_audit.json']),
        ('a8_plot', 'scripts/plot_support_sensitivity.py', [*navargs, '--run-dir', a8, '--a7-run-dir', a7, '--audit-json', out/'a8_audit.json', '--output-dir', out/'a8_figure']),
        ('a9_run', 'scripts/run_support_optimization.py', [*navargs, '--a8-run-dir', a8, '--a8-audit-json', out/'a8_audit.json', '--a7-run-dir', a7, '--a7-audit-json', out/'a7_audit.json', '--output-dir', a9]),
        ('a9_audit', 'research_tests/audit_support_optimization.py', [*navargs, '--run-dir', a9, '--a8-run-dir', a8, '--a8-audit-json', out/'a8_audit.json', '--a7-run-dir', a7, '--json-out', out/'a9_audit.json']),
        ('a9_plot', 'scripts/plot_support_optimization.py', [*navargs, '--run-dir', a9, '--a8-run-dir', a8, '--audit-json', out/'a9_audit.json', '--output-dir', out/'a9_figure']),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--archives-dir', type=Path, help='Directory containing the two pinned ENC ZIP snapshots')
    group.add_argument('--download', action='store_true', help='Download current official archives; stop if they differ from the fixed pins')
    parser.add_argument('--noaa-cname', action='store_true')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args(); out = args.output_dir.resolve()
    if args.archives_dir:
        records = verify_archives(args.archives_dir)
    if out.exists():
        raise FileExistsError('Use a new output directory; existing results are never overwritten')
    out.mkdir(parents=True); (out/'logs').mkdir()
    env = os.environ.copy(); env['PYTHONPATH'] = str(ROOT/'src') + os.pathsep + str(ROOT/'research_tests')
    # Existing Postgres.app is an optional local toolchain fallback, not an installation.
    native = Path('/Applications/Postgres.app/Contents/Versions/18/bin')
    if native.is_dir():
        env['PATH'] = env.get('PATH', '') + os.pathsep + str(native)
    manifest = {'status': 'running', 'started_utc': datetime.now(timezone.utc).isoformat(),
                'python': sys.version, 'platform': platform.platform(),
                'packages': {n: importlib.metadata.version(n) for n in ('numpy', 'scipy', 'Pillow', 'PyMuPDF', 'matplotlib')},
                'stages': [], 'input_mode': 'local_pinned_snapshots' if args.archives_dir else 'official_download',
                'code_config_sha256': {str(p.relative_to(ROOT)): digest(p) for folder in ('src', 'scripts', 'research_tests', 'configs')
                                       for p in sorted((ROOT/folder).rglob('*')) if p.suffix in ('.py', '.json')}}
    def execute(name, script, arguments):
        argv = [sys.executable, '-m' if script == '-m' else str(ROOT/script), *map(str, arguments)]
        row = {'name': name, 'command': argv, 'status': 'running', 'log': 'logs/'+name+'.log'}
        manifest['stages'].append(row); (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print(name + ': running', flush=True); started = time.perf_counter()
        with (out/row['log']).open('w') as log:
            process = subprocess.run(argv, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        row.update(returncode=process.returncode, elapsed_s=time.perf_counter()-started,
                   status='passed' if process.returncode == 0 else 'failed', log_sha256=digest(out/row['log']))
        if process.returncode:
            manifest['status'] = 'failed'
        (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        if process.returncode:
            raise SystemExit('Stopped at '+name+'; see '+str(out/row['log']))
        print(name + ': passed', flush=True)
    if args.download:
        download_args = ['--output-dir', out/'archives']
        if args.noaa_cname:
            download_args.append('--noaa-cname')
        execute('download', 'scripts/download_open_chart_stage_a.py', download_args)
        records = verify_archives(out/'archives')
    else:
        (out/'archives').mkdir()
        for name in records:
            shutil.copyfile(args.archives_dir/name, out/'archives'/name)
    manifest['chart_snapshots'] = records
    for name, script, arguments in commands(out):
        execute(name, script, arguments)
    execute('core_tests', '-m', ['unittest', 'discover', '-s', 'tests', '-q'])
    execute('research_tests', '-m', ['unittest', 'discover', '-s', 'research_tests', '-q'])
    # Acceptance is based on the newly produced audits, not historical output paths.
    audits = {stage: json.loads((out/(stage+'_audit.json')).read_text())['passed']
              for stage in ('a1', 'a2', 'a3', 'a5', 'a6', 'a6_repair', 'a7', 'a8', 'a9')}
    a4 = json.loads((out/'a4_study/study_summary.json').read_text())
    audits['a4_local_study'] = len(a4['cases']) == 6 and all(c['audit_passed'] for c in a4['cases'])
    x = json.loads((out/'a9/result.json').read_text()); audit = json.loads((out/'a9_audit.json').read_text())
    pairs = x['strategies']['scheduled_mobile']['pairs']
    acceptance = {'stage': 'A10', 'passed': all(audits.values()), 'audits': audits,
                  'target_area_m2': json.loads((out/'a1_map/summary.json').read_text())['searchable_area_m2'],
                  'target_geometry_sha256': digest(out/'a1_map/searchable.geojson'),
                  'hazards_geometry_sha256': digest(out/'a1_map/hazards.geojson'),
                  'navigation_geometry_sha256': digest(out/'a5_nav/navigation.geojson'),
                  'scan_windows': len(x['scan_windows']), 'mission_end_s': x['mission_end_s'],
                  'coverage_lower_ratio': x['coverage_lower_ratio_unchanged'], 'a9_comparison': x['comparison'],
                  'maximum_support_distance_upper_m': max(p['maximum_distance_upper_m'] for p in pairs.values()),
                  'minimum_usv_uuv_distance_lower_m': min(p['minimum_distance_lower_m'] for p in pairs.values()),
                  'independent_usv_hazard_clearance_lower_m': audit['mobile_hazard_clearance_lower_m'],
                  'strict_original_thesis_reproduction': False,
                  'scope': 'Fixed-chart offline 2D planning reproduction. No tracking controller, currents, energy, acoustics, deployment or real-system validation.'}
    pins = json.loads((ROOT/'configs/support_optimization_a9.json').read_text())
    acceptance['passed'] &= (acceptance['target_geometry_sha256'] == pins['target_geometry_sha256']
                              and acceptance['hazards_geometry_sha256'] == pins['hazards_geometry_sha256']
                              and acceptance['navigation_geometry_sha256'] == pins['navigation_sha256']
                              and acceptance['scan_windows'] == 89 and acceptance['coverage_lower_ratio'] >= .999
                              and acceptance['maximum_support_distance_upper_m'] <= 2300
                              and acceptance['minimum_usv_uuv_distance_lower_m'] >= 25
                              and acceptance['independent_usv_hazard_clearance_lower_m'] >= 50)
    acceptance['output_sha256'] = {str(p.relative_to(out)): digest(p) for p in sorted(out.rglob('*'))
                                  if p.is_file() and p != out/'manifest.json'}
    (out/'acceptance.json').write_text(json.dumps(acceptance, indent=2)+'\n')
    manifest.update(status='passed' if acceptance['passed'] else 'failed', acceptance_sha256=digest(out/'acceptance.json'),
                    finished_utc=datetime.now(timezone.utc).isoformat())
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps({k: v for k, v in acceptance.items() if k != 'output_sha256'}, indent=2))
    if not acceptance['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
