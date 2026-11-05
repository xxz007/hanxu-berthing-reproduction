"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.navigation import *
from berthing_planner.experiment.navigation import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/headland_navigation_a5.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--with-grid',action='store_true')
    args=parser.parse_args()
    return _run(prepared_dir=args.prepared_dir, config=args.config, output_dir=args.output_dir, with_grid=args.with_grid)


if __name__=="__main__":main()
