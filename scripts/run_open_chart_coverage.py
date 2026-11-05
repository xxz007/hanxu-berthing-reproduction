"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.coverage import *
from berthing_planner.experiment.coverage import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--grid-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/open_chart_coverage_a2.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    return _run(prepared_dir=args.prepared_dir, grid_dir=args.grid_dir, config=args.config, output_dir=args.output_dir)


if __name__=="__main__":main()
