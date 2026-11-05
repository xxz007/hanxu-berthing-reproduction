"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.routes import *
from berthing_planner.experiment.routes import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('a3-run-dir','a5-run-dir','prepared-dir','navigation-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/joint_coverage_a6.json')
    args=parser.parse_args()
    return _run(allocation_dir=args.a3_run_dir, candidates_dir=args.a5_run_dir, prepared_dir=args.prepared_dir, navigation_dir=args.navigation_dir, output_dir=args.output_dir, config=args.config)


if __name__=="__main__":main()
