"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.validation.chart import *
from berthing_planner.validation.chart import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir',type=Path,required=True)
    parser.add_argument('--json-out',type=Path,required=True)
    parser.add_argument('--proj-bin',type=Path,default=Path('/Applications/Postgres.app/Contents/Versions/18/bin/proj'))
    args=parser.parse_args()
    return _run(prepared_dir=args.prepared_dir, json_out=args.json_out, proj_bin=args.proj_bin)


if __name__=="__main__":main()
