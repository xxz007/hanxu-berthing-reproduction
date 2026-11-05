"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.replay import *
from berthing_planner.experiment.replay import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','audit-json','prepared-dir','navigation-dir','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    return _run(tracking_dir=args.run_dir, tracking_audit=args.audit_json, prepared_dir=args.prepared_dir, navigation_dir=args.navigation_dir, output=args.output)


if __name__=="__main__":main()
