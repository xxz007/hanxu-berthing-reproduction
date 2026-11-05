"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.control import *
from berthing_planner.experiment.control import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a9-run-dir','a9-audit-json','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/tracking_control_a11.json');parser.add_argument('--case',default='nominal');parser.add_argument('--step-s',type=float)
    args=parser.parse_args()
    return _run(reference_dir=args.a9_run_dir, reference_audit=args.a9_audit_json, prepared_dir=args.prepared_dir, navigation_dir=args.navigation_dir, output_dir=args.output_dir, config=args.config, case=args.case, step_s=args.step_s)


if __name__=="__main__":main()
