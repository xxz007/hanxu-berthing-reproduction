"""Final functional workflow: raw ENC → plan → feedback → conditions → evidence."""
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import shutil
import sys
import time
import urllib.request

from . import allocation, candidates, chart, contracts, control, coverage, figures, grid, navigation, replay, routes, scanning, schedule, support
from ..validation import allocation as check_allocation, chart as check_chart, control as check_control, coverage as check_coverage, reference as check_reference, routes as check_routes, scanning as check_scanning

ROOT = Path(__file__).resolve().parents[3]
SOURCE_PIN = 'source_reference_scientific_sha256'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def validate_config(cfg):
    """The entry boundary owns configuration checks; inner services use paths."""
    if cfg['schema_version'] != 1 or cfg['experiment'] != 'long_beach_cooperative_coverage' or cfg['grid_n'] != 2048:
        raise ValueError('This release reproduces the pinned Long Beach case with a 2048 grid')
    if set(cfg['components']) != {'chart', 'coverage', 'allocation', 'navigation', 'routes', 'scanning', 'support'}:
        raise ValueError('Complete named component profiles required')
    names = cfg['control']['design']['case_order']
    if len(names) != len(set(names)) or set(names) != set(cfg['control']['cases']) or names[0] != 'nominal':
        raise ValueError('Registered conditions must be unique, complete and start with nominal')
    if cfg['periods'] != [{'name':'nominal','step_s':.5},{'name':'period_025','step_s':.25},{'name':'period_100','step_s':1.}]:
        raise ValueError('The fixed verification periods are 0.5, 0.25 and 1 s')
    if cfg['control']['step_s'] != .5:
        raise ValueError('Registered conditions use the same 0.5 s controller')
    paths = {n: ROOT/p for n, p in cfg['components'].items()}
    for path in (*paths.values(), ROOT/cfg['regression']):
        if not path.is_file():
            raise FileNotFoundError(path)
    return paths


def verify_archives(directory, chart_config):
    records = {}
    for item in chart_config['charts']:
        path = directory/(item['cell']+'.zip')
        if not path.is_file() or path.stat().st_size != item['bytes'] or digest(path) != item['sha256']:
            raise ValueError('Pinned ENC snapshot missing or changed: '+item['cell'])
        records[path.name] = dict(sha256=digest(path), bytes=path.stat().st_size, url=item['url'])
    return records


def run(*, config, output_dir, archives_dir=None, qa_script_dir=None):
    cfg = read(config); components = validate_config(cfg)
    chart_config = read(components['chart'])
    if archives_dir is not None:
        raw_records = verify_archives(archives_dir, chart_config)
    out = output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    for name in ('logs', 'configuration', 'validation', 'report'):
        (out/name).mkdir()
    manifest = dict(schema_version=1, experiment=cfg['experiment'], status='running', started_utc=datetime.now(timezone.utc).isoformat(), python=sys.version, platform=platform.platform(), packages={n:importlib.metadata.version(n) for n in ('numpy','scipy','Pillow','PyMuPDF','matplotlib')}, steps=[], source_sha256={str(p.relative_to(ROOT)):digest(p) for folder in ('src','scripts','research_tests','configs') for p in sorted((ROOT/folder).rglob('*')) if p.suffix in ('.py','.json')}, entry_sha256=digest(ROOT/'run.py'))
    write(out/'manifest.json', manifest)

    def execute(name, function, **kwargs):
        row = dict(name=name, status='running', log='logs/'+name+'.log')
        manifest['steps'].append(row); write(out/'manifest.json', manifest)
        print(name+': running', flush=True); started=time.perf_counter()
        with (out/row['log']).open('w') as log, redirect_stdout(log), redirect_stderr(log):
            function(**kwargs)
        row.update(status='passed', elapsed_s=time.perf_counter()-started, log_sha256=digest(out/row['log']))
        write(out/'manifest.json', manifest); print(name+': passed', flush=True)

    try:
        shutil.copy2(config, out/'configuration/experiment.json')
        for name, source in components.items():
            target = out/'configuration'/(name+'.json'); shutil.copy2(source, target); components[name] = target
        shutil.copy2(ROOT/cfg['regression'], out/'configuration/regression.json')
        raw = out/'inputs'; raw.mkdir()
        if archives_dir is None:
            for item in chart_config['charts']:
                with urllib.request.urlopen(item['url'], timeout=45) as response:
                    (raw/(item['cell']+'.zip')).write_bytes(response.read())
            raw_records = verify_archives(raw, chart_config)
        else:
            for name in raw_records:
                shutil.copy2(archives_dir/name, raw/name)
        write(raw/'sources.json', dict(retrieved_utc=datetime.now(timezone.utc).isoformat(), mode='verified_local_snapshot' if archives_dir else 'official_download', archives=raw_records))
        manifest['raw_inputs'] = raw_records
        prepared, raster, nav = (out/'map'/n for n in ('chart','grid','navigation'))
        cover, allocated, layers, curves, scans, timing, reference = (out/'planning'/n for n in ('coverage','allocation','candidates','curves','scans','schedule','reference'))
        checks = out/'validation'
        execute('chart', chart.run, config=components['chart'], archives_dir=raw, output_dir=prepared)
        execute('chart_check', check_chart.run, prepared_dir=prepared, json_out=checks/'chart.json')
        execute('grid', grid.run, prepared_dir=prepared, output_dir=raster, n=cfg['grid_n'])
        execute('coverage', coverage.run, prepared_dir=prepared, grid_dir=raster, config=components['coverage'], output_dir=cover)
        execute('coverage_check', check_coverage.run, coverage_dir=cover, prepared_dir=prepared, grid_dir=raster, json_out=checks/'coverage.json')
        effective_allocation = out/'configuration/allocation_effective.json'
        execute('coverage_contract', contracts.run, coverage_dir=cover, reference_config=components['allocation'], output_config=effective_allocation)
        execute('allocation', allocation.run, coverage_dir=cover, prepared_dir=prepared, grid_dir=raster, config=effective_allocation, output_dir=allocated)
        execute('allocation_check', check_allocation.run, allocation_dir=allocated, coverage_dir=cover, prepared_dir=prepared, grid_dir=raster, json_out=checks/'allocation.json')
        execute('navigation', navigation.run, prepared_dir=prepared, config=components['navigation'], output_dir=nav)
        execute('scan_candidates', candidates.run, allocation_dir=allocated, prepared_dir=prepared, navigation_dir=nav, config=components['navigation'], output_dir=layers)
        execute('curves', routes.run, allocation_dir=allocated, candidates_dir=layers, prepared_dir=prepared, navigation_dir=nav, output_dir=curves, config=components['routes'])
        execute('curves_check', check_routes.run, routes_dir=curves, allocation_dir=allocated, candidates_dir=layers, prepared_dir=prepared, navigation_dir=nav, json_out=checks/'curves.json')
        execute('scan_schedule', scanning.run, routes_dir=curves, route_audit=checks/'curves.json', prepared_dir=prepared, output_dir=scans, config=components['scanning'])
        execute('scan_check', check_scanning.run, scans_dir=scans, routes_dir=curves, route_audit=checks/'curves.json', prepared_dir=prepared, json_out=checks/'scans.json')
        execute('common_clock', schedule.run, routes_dir=curves, scans_dir=scans, route_audit=checks/'curves.json', scan_audit=checks/'scans.json', config=components['support'], output_dir=timing)
        execute('clock_check', check_reference.audit_schedule, schedule_dir=timing, routes_dir=curves, scans_dir=scans, config=components['support'], json_out=timing/'audit.json')
        execute('cooperative_support', support.run, seed_dir=timing, prepared_dir=prepared, navigation_dir=nav, config=components['support'], output_dir=reference)
        execute('reference_check', check_reference.run, reference_dir=reference, schedule_dir=timing, prepared_dir=prepared, navigation_dir=nav, json_out=checks/'reference.json')
        effective_control = out/'configuration/control_effective.json'
        control_cfg = cfg['control'] | {SOURCE_PIN: control.canonical({k:v for k,v in read(reference/'result.json').items() if k!='total_computation_seconds'})}
        write(effective_control, control_cfg)
        period_audits = []
        for period in cfg['periods']:
            name = period['name']; run_dir=out/'control'/name; audit=checks/('control_'+name+'.json')
            execute('control_'+name, control.run, reference_dir=reference, reference_audit=checks/'reference.json', prepared_dir=prepared, navigation_dir=nav, output_dir=run_dir, config=effective_control, step_s=period['step_s'], source_pin_key=SOURCE_PIN)
            execute('control_'+name+'_check', check_control.run, tracking_dir=run_dir, reference_dir=reference, prepared_dir=prepared, navigation_dir=nav, json_out=audit, source_pin_key=SOURCE_PIN)
            period_audits.append(read(audit))
        conditions = out/'conditions'; conditions.mkdir(); rows=[]
        for name in control_cfg['design']['case_order']:
            run_dir = out/'control/nominal' if name=='nominal' else conditions/name
            audit = checks/('control_nominal.json' if name=='nominal' else 'condition_'+name+'.json')
            if name != 'nominal':
                execute('condition_'+name, control.run, reference_dir=reference, reference_audit=checks/'reference.json', prepared_dir=prepared, navigation_dir=nav, output_dir=run_dir, config=effective_control, case=name, source_pin_key=SOURCE_PIN)
                execute('condition_'+name+'_check', check_control.run, tracking_dir=run_dir, reference_dir=reference, prepared_dir=prepared, navigation_dir=nav, json_out=audit, source_pin_key=SOURCE_PIN)
            x, a = read(run_dir/'result.json'), read(audit); pairs=a['distance_pairs']
            rows.append(dict(case=name, audit_passed=a['passed'], constraint_outcome=a['constraint_outcome'], maximum_position_error_m=max(v['maximum_position_error_m'] for v in x['metrics'].values()), coverage_lower_ratio=a['independent_coverage_lower_ratio'], coverage_upper_ratio=a['independent_coverage_upper_ratio'], minimum_pair_distance_lower_m=min(v['minimum_distance_lower_m'] for v in pairs.values()), maximum_support_distance_upper_m=max(v['maximum_distance_upper_m'] for k,v in pairs.items() if k.startswith('usv__')), minimum_hazard_clearance_lower_m=min(a['independent_hazard_clearance_lower_m'].values()), source_result_sha256=digest(run_dir/'result.json'), run_dir=str(run_dir), audit_json=str(audit)))
        study = dict(kind='registered_conditions', passed=all(r['audit_passed'] for r in rows), cases=rows, design=control_cfg['design'], source_nominal_result_sha256=digest(out/'control/nominal/result.json'), scope=control_cfg['scope'])
        write(conditions/'study_summary.json', study)
        execute('figures', figures.run, tracking_dir=out/'control/nominal', tracking_audit=checks/'control_nominal.json', conditions_dir=conditions, prepared_dir=prepared, navigation_dir=nav, output_dir=out/'figures', qa_script_dir=qa_script_dir)
        execute('replay', replay.run, tracking_dir=out/'control/nominal', tracking_audit=checks/'control_nominal.json', prepared_dir=prepared, navigation_dir=nav, output=out/'report/replay.html')
        from ..validation.delivery import run as audit_delivery
        execute('delivery_check', audit_delivery, run_dir=out, json_out=checks/'delivery.json')
        delivery = read(checks/'delivery.json')
        accepted = delivery['passed'] and all(a['passed'] and a['mission_constraints_certified'] for a in period_audits) and study['passed']
        summary = dict(experiment=cfg['experiment'], passed=accepted, mission_end_s=read(reference/'result.json')['mission_end_s'], scan_count=89, usv_transfer_count=len(read(reference/'result.json')['mobile_usv']['legs']), usv_length_m=read(reference/'result.json')['mobile_usv']['length_m'], nominal_audit=period_audits[0], conditions=[{k:v for k,v in r.items() if k not in ('run_dir','audit_json','source_result_sha256')} for r in rows], strict_original_thesis_reproduction=False, real_marine_vehicle_validated=False, learner_or_supervisor_acceptance=False, scope=control_cfg['scope'])
        write(out/'report/summary.json', summary)
        from .report import render
        (out/'report/results.md').write_text(render(summary))
        manifest.update(status='passed' if accepted else 'failed', finished_utc=datetime.now(timezone.utc).isoformat())
        if not accepted:
            raise ValueError('Integrated scientific acceptance did not pass')
    finally:
        if manifest['status']=='running':
            manifest['status']='failed'
            if manifest['steps'] and manifest['steps'][-1]['status']=='running':
                manifest['steps'][-1]['status']='failed'
        manifest['output_sha256']={str(p.relative_to(out)):digest(p) for p in sorted(out.rglob('*')) if p.is_file() and p!=out/'manifest.json'}
        write(out/'manifest.json', manifest)
    print('Integrated experiment passed; report:', out/'report/results.md', flush=True)
    return summary
