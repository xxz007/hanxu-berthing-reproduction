"""A12 registered matched-reference scenarios; retain audited constraint failures."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a11-run-dir','a11-audit-json','a9-run-dir','a9-audit-json','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/tracking_robustness_a12.json');args=parser.parse_args();read=lambda f:json.loads(f.read_text());cfg=read(args.config);base=read(args.a11_run_dir/'result.json');audit=read(args.a11_audit_json);basecfg=read(args.a11_run_dir/'config.json');digest=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    if not audit['passed']or not audit['mission_constraints_certified']or audit['source_result_sha256']!=digest(args.a11_run_dir/'result.json'):raise ValueError('Audited passed A11 nominal run required before A12')
    excluded=('stage','cases','design')
    if {k:v for k,v in cfg.items()if k not in excluded}!={k:v for k,v in basecfg.items()if k not in excluded}or base['case']!='nominal'or base['effective_step_s']!=cfg['step_s']:raise ValueError('A12 must freeze A11 model, gains, limits, step and mission gates')
    if cfg['cases']['nominal']!=base['scenario']:raise ValueError('Nominal baseline changed')
    if set(cfg['design']['case_order'])!=set(cfg['cases'])or len(cfg['design']['case_order'])!=len(cfg['cases']):raise ValueError('Registered cases must be complete and unique')
    args.output_dir.mkdir(parents=True,exist_ok=False);(args.output_dir/'config.json').write_bytes(args.config.read_bytes());(args.output_dir/'logs').mkdir();rows=[]
    def execute(command,name):
        with(args.output_dir/'logs'/(name+'.log')).open('w')as f:subprocess.run([str(v)for v in command],stdout=f,stderr=subprocess.STDOUT,check=True)
    for name in cfg['design']['case_order']:
        if name=='nominal':run=args.a11_run_dir;auditfile=args.a11_audit_json
        else:
            run=args.output_dir/'cases'/name;auditfile=args.output_dir/(name+'_audit.json')
            common=['--a9-run-dir',args.a9_run_dir,'--prepared-dir',args.prepared_dir,'--navigation-dir',args.navigation_dir]
            execute([sys.executable,ROOT/'scripts/run_tracking_control.py',*common,'--a9-audit-json',args.a9_audit_json,'--config',args.config,'--case',name,'--output-dir',run],name)
            execute([sys.executable,ROOT/'research_tests/audit_tracking_control.py',*common,'--run-dir',run,'--json-out',auditfile],name+'_audit')
        x=read(run/'result.json');a=read(auditfile)
        if not a['passed']or a['source_result_sha256']!=digest(run/'result.json'):raise ValueError('Matched mathematical audit required for every case')
        pairs=a['distance_pairs'];rows.append({'case':name,'audit_passed':a['passed'],'constraint_outcome':a['constraint_outcome'],'maximum_position_error_m':max(v['maximum_position_error_m']for v in x['metrics'].values()),'coverage_lower_ratio':a['independent_coverage_lower_ratio'],'coverage_upper_ratio':a['independent_coverage_upper_ratio'],'minimum_pair_distance_lower_m':min(v['minimum_distance_lower_m']for v in pairs.values()),'maximum_support_distance_upper_m':max(v['maximum_distance_upper_m']for k,v in pairs.items()if k.startswith('usv__')),'minimum_hazard_clearance_lower_m':min(a['independent_hazard_clearance_lower_m'].values()),'source_result_sha256':digest(run/'result.json'),'run_dir':str(run.resolve()),'audit_json':str(auditfile.resolve())})
        print(name,a['constraint_outcome'],flush=True)
    result={'stage':'A12','passed':all(r['audit_passed']for r in rows)and rows[0]['constraint_outcome']=='certified_pass','cases':rows,'design':cfg['design'],'source_nominal_result_sha256':digest(args.a11_run_dir/'result.json'),'scope':cfg['scope']}
    (args.output_dir/'study_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items()if k not in('cases','scope')},indent=2))
    if not result['passed']:raise SystemExit(1)


if __name__=='__main__':main()
