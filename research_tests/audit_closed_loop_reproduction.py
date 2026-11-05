"""A13 complete fresh-data chain, retained failures and artifact binding audit."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from run_closed_loop_reproduction import PERIODS,completion_gate


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True);parser.add_argument('--json-out',type=Path,required=True);args=parser.parse_args();out=args.run_dir.resolve()
    read=lambda f:json.loads(f.read_text());digest=lambda f:hashlib.sha256(f.read_bytes()).hexdigest();m=read(out/'manifest.json');x=read(out/'acceptance.json');errors=[]
    if m['status']!='passed'or not x['passed']or digest(out/'acceptance.json')!=m['acceptance_sha256']:errors.append('A13 acceptance binding')
    for rel,h in m['code_config_sha256'].items():
        if digest(ROOT/rel)!=h:errors.append('code/config drift: '+rel)
    for rel,h in x['output_sha256'].items():
        if digest(out/rel)!=h:errors.append('output hash: '+rel)
    expected=['planning','planning_audit']+[p for label,_ in PERIODS for p in('a11_'+label,'a11_'+label+'_audit')]+['a12','figures','replay']
    if [r['name']for r in m['steps']]!=expected:errors.append('stage execution order')
    for row in m['steps']:
        if row['returncode']or row['status']!='passed'or digest(out/row['log'])!=row['log_sha256']:errors.append('stage failed or log changed')
    if not read(out/'planning_audit.json')['passed']or read(out/'planning/manifest.json')['status']!='passed':errors.append('raw-chart planning acceptance')
    periods=[]
    for label,step in PERIODS:
        run=out/'a11'/label;a=read(out/'a11'/(label+'_audit.json'));r=read(run/'result.json');periods.append(a)
        if a['source_result_sha256']!=digest(run/'result.json')or r['effective_step_s']!=step:errors.append('period experiment binding')
        proof=read(run/'provenance.json')
        if str((out/'planning/a9/result.json').resolve())not in proof['input_sha256']:errors.append('old planning output reused')
    study=read(out/'a12/study_summary.json');cfg=read(out/'a12/config.json');outcomes={r['case']:r['constraint_outcome']for r in study['cases']}
    if list(outcomes)!=cfg['design']['case_order']or len(outcomes)!=7:errors.append('registered case accounting')
    for row in study['cases']:
        run=Path(row['run_dir']).resolve();auditpath=Path(row['audit_json']).resolve()
        if not run.is_relative_to(out)or not auditpath.is_relative_to(out):errors.append('external old control output reused')
        a=read(auditpath);r=read(run/'result.json')
        if not a['passed']or a['source_result_sha256']!=digest(run/'result.json')or row['source_result_sha256']!=a['source_result_sha256']or row['constraint_outcome']!=a['constraint_outcome']or r['scenario']!=cfg['cases'][row['case']]:errors.append('scenario audit/result binding')
    if not completion_gate(periods,study)or x['case_outcomes']!=outcomes:errors.append('final stage gate or outcome labels')
    if x['all_registered_conditions_safe']!=all(v=='certified_pass'for v in outcomes.values()):errors.append('retained violations hidden')
    report={'passed':not errors,'errors':errors,'outer_steps':len(m['steps']),'planning_steps':x['planning_steps'],'output_files_hash_checked':len(x['output_sha256']),'period_cases':len(periods),'registered_disturbance_cases':len(outcomes),'case_outcomes':outcomes,'all_registered_conditions_safe':x['all_registered_conditions_safe'],'acceptance_sha256':digest(out/'acceptance.json'),'model_scope':x['scope'],'strict_original_thesis_reproduction':False,'learner_or_supervisor_acceptance':False}
    args.json_out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if errors:raise SystemExit(1)


if __name__=='__main__':main()
