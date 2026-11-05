"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.validation.allocation import *
from berthing_planner.validation.allocation import run as _run

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('run-dir', 'a2-run-dir', 'prepared-dir', 'grid-dir', 'json-out'):
        parser.add_argument('--'+key, type=Path, required=True)
    args = parser.parse_args()
    return _run(allocation_dir=args.run_dir, coverage_dir=args.a2_run_dir, prepared_dir=args.prepared_dir, grid_dir=args.grid_dir, json_out=args.json_out)


if __name__=="__main__":main()
