"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.figures import *
from berthing_planner.experiment.figures import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a11-run-dir','a11-audit-json','a12-study-dir','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--qa-script-dir',type=Path);args=parser.parse_args()
    return _run(tracking_dir=args.a11_run_dir, tracking_audit=args.a11_audit_json, conditions_dir=args.a12_study_dir, prepared_dir=args.prepared_dir, navigation_dir=args.navigation_dir, output_dir=args.output_dir, qa_script_dir=args.qa_script_dir)


if __name__=="__main__":main()
