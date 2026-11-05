"""Artifact-chain and frozen-science regression gate for the final experiment."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',',':'), ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def run(*, run_dir, json_out, final=False):
    read=lambda p:json.loads(p.read_text())
    manifest=read(run_dir/'manifest.json'); cfg=read(run_dir/'configuration/experiment.json'); baseline=read(run_dir/'configuration/regression.json'); errors=[]
    if digest(run_dir/'configuration/regression.json')!=digest(ROOT/cfg['regression']):
        errors.append('external regression fixture binding')
    for path, pin in manifest['source_sha256'].items():
        if digest(ROOT/path)!=pin:
            errors.append('code/config changed during run: '+path)
    if digest(ROOT/'run.py')!=manifest['entry_sha256']:
        errors.append('entry changed during run')
    for name, item in manifest['raw_inputs'].items():
        p=run_dir/'inputs'/name
        if digest(p)!=item['sha256'] or p.stat().st_size!=item['bytes']:
            errors.append('raw archive binding')
    reference=read(run_dir/'planning/reference/result.json')
    if canonical({k:reference[k] for k in baseline['reference_fields']})!=baseline['reference_scientific_sha256']:
        errors.append('reference scientific regression')
    outputs={p['name']:run_dir/'control'/p['name'] for p in cfg['periods']}
    outputs.update({n:run_dir/'conditions'/n for n in cfg['control']['design']['case_order'] if n!='nominal'})
    hashes=0
    if set(outputs)!=set(baseline['derived_files_sha256']):
        errors.append('complete simulation set')
    for name, pins in baseline['derived_files_sha256'].items():
        for filename, pin in pins.items():
            if digest(outputs[name]/filename)!=pin:
                errors.append('scientific derived regression: '+name+'/'+filename)
            hashes+=1
    study=read(run_dir/'conditions/study_summary.json')
    outcomes={r['case']:r['constraint_outcome'] for r in study['cases']}
    if outcomes!=baseline['case_outcomes'] or not study['passed'] or len(study['cases'])!=7:
        errors.append('registered pass/violation outcomes')
    for row in study['cases']:
        audit=read(Path(row['audit_json']))
        if not audit['passed'] or audit['source_result_sha256']!=digest(Path(row['run_dir'])/'result.json') or row['source_result_sha256']!=audit['source_result_sha256'] or row['constraint_outcome']!=audit['constraint_outcome']:
            errors.append('condition audit chain')
    checks=('chart','coverage','allocation','curves','scans','reference','control_nominal','control_period_025','control_period_100')
    for name in checks:
        check=read(run_dir/'validation'/(name+'.json'))
        if not check['passed']:
            errors.append('independent check failed: '+name)
    nominal=read(run_dir/'control/nominal/result.json')
    if len(reference['scan_windows'])!=baseline['scan_count'] or abs(nominal['geometry']['target_area_m2']-baseline['target_area_m2'])>1e-6:
        errors.append('full target/scan inheritance')
    # All saved producer provenance is checked, including local intermediates.
    provenances=0
    for path in run_dir.rglob('provenance.json'):
        proof=read(path)
        for key in ('input_sha256','code_sha256','input_and_code_sha256'):
            for relative,pin in proof.get(key,{}).items():
                target=ROOT/relative if key=='code_sha256' else Path(relative)
                if digest(target)!=pin:
                    errors.append('provenance '+str(path.relative_to(run_dir))+'/'+relative)
        for key in ('derived_sha256','derived_files_sha256','geometry_sha256'):
            for relative,pin in proof.get(key,{}).items():
                if digest(path.parent/relative)!=pin:
                    errors.append('derived provenance '+str(path.relative_to(run_dir))+'/'+relative)
        if 'result_sha256' in proof and digest(path.parent/'result.json')!=proof['result_sha256']:
            errors.append('result provenance '+str(path.relative_to(run_dir)))
        provenances+=1
    allowed={'delivery_check'} if not final else set()
    if any(s['status']!='passed' and s['name'] not in allowed for s in manifest['steps']):
        errors.append('incomplete functional workflow')
    output_hashes=0
    if final:
        if manifest['status']!='passed':errors.append('final manifest status')
        for relative,pin in manifest['output_sha256'].items():
            if digest(run_dir/relative)!=pin:
                errors.append('final artifact hash: '+relative)
            output_hashes+=1
    report=dict(passed=not errors,errors=errors,reference_science_identical=not any('reference scientific' in e for e in errors),scientific_derived_files_checked=hashes,provenance_files_checked=provenances,final_output_hashes_checked=output_hashes,case_outcomes=outcomes,scope='Artifact and fixed scientific regression acceptance; retained adverse conditions are not certified safe.')
    json_out.write_text(json.dumps(report,indent=2)+'\n')
    if errors:
        raise ValueError('Integrated delivery audit: '+', '.join(errors[:15]))
