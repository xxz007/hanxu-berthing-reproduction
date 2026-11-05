"""A13 raw-ENC to planning, feedback control and registered disturbance study.

Preserves the original A10 entry point. This extended entry installs nothing,
requires a fresh directory and retains adverse cases as scientific results.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from run_reproduction import verify_archives

ROOT=Path(__file__).resolve().parents[1]
PERIODS=(('nominal',.5),('period_025',.25),('period_100',1.))


def completion_gate(period_audits,study):
    return (len(period_audits)==3 and all(a['passed']and a['mission_constraints_certified']for a in period_audits)
            and study['passed'] and len(study['cases'])==7
            and all(r['audit_passed']for r in study['cases'])
            and next(r['constraint_outcome']for r in study['cases']if r['case']=='nominal')=='certified_pass'
            and next(r['constraint_outcome']for r in study['cases']if r['case']=='current_over_limit')=='witnessed_violation')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True);group.add_argument('--archives-dir',type=Path);group.add_argument('--download',action='store_true')
    parser.add_argument('--noaa-cname',action='store_true');parser.add_argument('--output-dir',type=Path,required=True);parser.add_argument('--qa-script-dir',type=Path)
    args=parser.parse_args();out=args.output_dir.resolve()
    if args.archives_dir:verify_archives(args.archives_dir)
    out.mkdir(parents=True,exist_ok=False);(out/'logs').mkdir();digest=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    manifest={'stage':'A13','status':'running','started_utc':datetime.now(timezone.utc).isoformat(),'steps':[],'code_config_sha256':{str(f.relative_to(ROOT)):digest(f)for d in ('src','scripts','research_tests','configs')for f in sorted((ROOT/d).rglob('*'))if f.suffix in('.py','.json')}}
    def write_manifest():(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    env=os.environ.copy();env['PYTHONPATH']=str(ROOT/'src')+os.pathsep+str(ROOT/'research_tests')
    def execute(name,script,argv):
        command=[sys.executable,str(ROOT/script),*map(str,argv)];row={'name':name,'command':command,'log':'logs/'+name+'.log','status':'running'};manifest['steps'].append(row);write_manifest();print(name+': running',flush=True)
        with(out/row['log']).open('w')as f:run=subprocess.run(command,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
        row.update(returncode=run.returncode,status='passed'if run.returncode==0 else'failed',log_sha256=digest(out/row['log']));write_manifest()
        if run.returncode:
            manifest['status']='failed';write_manifest();raise SystemExit('Stopped at '+name+'; see '+str(out/row['log']))
        print(name+': passed',flush=True)
    planning=out/'planning';raw=['--archives-dir',args.archives_dir]if args.archives_dir else['--download']
    if args.noaa_cname:raw.append('--noaa-cname')
    execute('planning','scripts/run_reproduction.py',[*raw,'--output-dir',planning])
    execute('planning_audit','research_tests/audit_reproduction.py',['--run-dir',planning,'--json-out',out/'planning_audit.json'])
    common=['--a9-run-dir',planning/'a9','--prepared-dir',planning/'a1_map','--navigation-dir',planning/'a5_nav']
    periods=[]
    for label,step in PERIODS:
        run=out/'a11'/label;audit=out/'a11'/(label+'_audit.json')
        execute('a11_'+label,'scripts/run_tracking_control.py',[*common,'--a9-audit-json',planning/'a9_audit.json','--step-s',step,'--output-dir',run])
        execute('a11_'+label+'_audit','research_tests/audit_tracking_control.py',[*common,'--run-dir',run,'--json-out',audit]);periods.append(json.loads(audit.read_text()))
    execute('a12','scripts/run_tracking_robustness.py',[*common,'--a9-audit-json',planning/'a9_audit.json','--a11-run-dir',out/'a11/nominal','--a11-audit-json',out/'a11/nominal_audit.json','--output-dir',out/'a12'])
    plot=['--a11-run-dir',out/'a11/nominal','--a11-audit-json',out/'a11/nominal_audit.json','--a12-study-dir',out/'a12','--prepared-dir',planning/'a1_map','--navigation-dir',planning/'a5_nav','--output-dir',out/'figures']
    if args.qa_script_dir:plot.extend(['--qa-script-dir',args.qa_script_dir])
    execute('figures','scripts/plot_control_results.py',plot)
    execute('replay','scripts/build_tracking_replay.py',['--run-dir',out/'a11/nominal','--audit-json',out/'a11/nominal_audit.json','--prepared-dir',planning/'a1_map','--navigation-dir',planning/'a5_nav','--output',out/'actual_tracking_replay.html'])
    study=json.loads((out/'a12/study_summary.json').read_text());accepted=completion_gate(periods,study)
    counts={name:int(re.search(r'Ran (\d+) tests',(planning/'logs'/(name+'.log')).read_text()).group(1))for name in ('core_tests','research_tests')}
    acceptance={'stage':'A13','passed':accepted,'stage_completion':{'A11':all(a['passed']and a['mission_constraints_certified']for a in periods),'A12':study['passed'],'A13':accepted},'planning_steps':len(json.loads((planning/'manifest.json').read_text())['stages']),'outer_steps':len(manifest['steps']),'tests':counts,'periods_s':[p for _,p in PERIODS],'case_outcomes':{r['case']:r['constraint_outcome']for r in study['cases']},'all_registered_conditions_safe':all(r['constraint_outcome']=='certified_pass'for r in study['cases']),'nominal_audit':{k:v for k,v in periods[0].items()if k not in('errors',)},'strict_original_thesis_reproduction':False,'real_marine_vehicle_validated':False,'learner_or_supervisor_acceptance':False,'scope':'Fixed-reference fully actuated planar ZOH acceleration surrogate. Registered deterministic disturbance cases, including retained violations. Not underactuated hydrodynamics, MPC, energy/acoustics/3D or real-system/learner acceptance.'}
    acceptance['output_sha256']={str(f.relative_to(out)):digest(f)for f in sorted(out.rglob('*'))if f.is_file()and f!=out/'manifest.json'}
    (out/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n');manifest.update(status='passed'if accepted else'failed',acceptance_sha256=digest(out/'acceptance.json'),finished_utc=datetime.now(timezone.utc).isoformat());write_manifest()
    if not accepted:raise SystemExit('Closed-loop stage gate not passed')
    command=[sys.executable,str(ROOT/'research_tests/audit_closed_loop_reproduction.py'),'--run-dir',str(out),'--json-out',str(out/'independent_acceptance.json')]
    with(out/'final_audit.log').open('w')as f:run=subprocess.run(command,cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
    if run.returncode:manifest['status']='failed';write_manifest();raise SystemExit('A13 final artifact audit failed')
    manifest['independent_audit_sha256']=digest(out/'independent_acceptance.json');write_manifest();print(json.dumps({k:v for k,v in acceptance.items()if k not in('output_sha256','nominal_audit')},indent=2))


if __name__=='__main__':main()
