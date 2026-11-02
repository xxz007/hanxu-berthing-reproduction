"""Reproduce all six declared A4 local-turn cases, independent audits and figure.

Successful execution means the feasibility study completed. It does not mean
the unchanged full A3 mission passed finite-radius motion acceptance.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a3-run-dir',type=Path,required=True)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--grid-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/motion_feasibility_a4.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path)
    args=parser.parse_args();out=args.output_dir.resolve()
    cfg=json.loads(args.config.read_text())
    if cfg['nominal_uuv_minimum_radius_m']!=20 or cfg['local_scan_spacing_m']!=80:
        raise ValueError('This study fixes20m nominal minimum and80m row spacing; use the single-case runner for other designs')
    out.mkdir(parents=True,exist_ok=False);(out/'configs').mkdir()
    rows=[]
    def execute(command,name):
        with(out/(name+'.log')).open('w')as log:
            subprocess.run([str(x)for x in command],stdout=log,stderr=subprocess.STDOUT,check=True)
    for radius in (20,40):
        for margin in (2.5,5,10):
            name=f'r{radius}_m{margin:g}';config=out/'configs'/(name+'.json')
            config.write_text(json.dumps({**cfg,'local_headland_radius_m':radius,'local_headland_extra_inset_m':margin},ensure_ascii=False,indent=2)+'\n')
            run=out/'cases'/name;audit=out/(name+'_audit.json')
            execute([sys.executable,ROOT/'scripts/run_motion_feasibility.py','--a3-run-dir',args.a3_run_dir,
                     '--prepared-dir',args.prepared_dir,'--config',config,'--output-dir',run],name)
            execute([sys.executable,ROOT/'research_tests/audit_motion_feasibility.py','--run-dir',run,
                     '--a3-run-dir',args.a3_run_dir,'--prepared-dir',args.prepared_dir,'--grid-dir',args.grid_dir,
                     '--json-out',audit],name+'_audit')
            result=json.loads((run/'result.json').read_text());patch=result['local_patch'];coverage=result['counterfactual_coverage']
            rows.append({'case':name,'turn_radius_m':radius,'extra_headland_margin_m':margin,
                         'local_duration_s':patch['profile']['duration_s'],'local_length_m':patch['profile']['length_m'],
                         'scan_off_coverage_ratio':coverage['after_one_patch_coverage_ratio'],
                         'scan_on_coverage_ratio_lower_bound':coverage['turn_connector_scanning_on_coverage_ratio_lower_bound'],
                         'scan_on_coverage_ratio_upper_bound':coverage['turn_connector_scanning_on_coverage_ratio_upper_bound'],
                         'audit_passed':json.loads(audit.read_text())['passed'],
                         'result_sha256':hashlib.sha256((run/'result.json').read_bytes()).hexdigest()})
            print(name,'independent audit completed',flush=True)
    command=[sys.executable,ROOT/'scripts/plot_motion_feasibility.py','--run-dir',out/'cases/r20_m10',
             '--audit-json',out/'r20_m10_audit.json','--comparison-run-dir',out/'cases/r40_m10',
             '--comparison-audit-json',out/'r40_m10_audit.json','--prepared-dir',args.prepared_dir,
             '--output-dir',out/'figure']
    if args.qa_script_dir:command.extend(['--qa-script-dir',args.qa_script_dir])
    execute(command,'figure')
    scripts=[Path(__file__),ROOT/'scripts/run_motion_feasibility.py',ROOT/'scripts/plot_motion_feasibility.py',
             ROOT/'research_tests/audit_motion_feasibility.py',ROOT/'src/berthing_planner/motion_feasibility.py']
    (out/'study_summary.json').write_text(json.dumps({'status':'six_case_study_completed','cases':rows,
        'full_mission_motion_feasible':False,'full_mission_retiming_released':False,
        'source_code_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in scripts},
        'interpretation':'Deterministic local radius/margin/sensor-policy comparisons, not independent statistical replicates or a complete repaired mission'},indent=2)+'\n')
    print(out/'study_summary.json')


if __name__=='__main__':main()
