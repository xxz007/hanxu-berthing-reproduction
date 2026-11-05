"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.validation.control import *
from berthing_planner.validation.control import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','a9-run-dir','prepared-dir','navigation-dir','json-out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    return _run(tracking_dir=args.run_dir, reference_dir=args.a9_run_dir, prepared_dir=args.prepared_dir, navigation_dir=args.navigation_dir, json_out=args.json_out)


if __name__=="__main__":main()
