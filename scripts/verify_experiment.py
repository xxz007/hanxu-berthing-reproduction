"""Check a completed integrated run, without recomputing its trajectories."""
import argparse
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from berthing_planner.validation.delivery import run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--json-out',type=Path,required=True,help='Use a new path outside the completed run directory')
    args=parser.parse_args()
    if args.json_out.resolve().is_relative_to(args.run_dir.resolve()) or args.json_out.exists():
        raise ValueError('Keep the completed run immutable; use a new external JSON path')
    run(run_dir=args.run_dir.resolve(),json_out=args.json_out,final=True)
    print('Integrated artifacts and scientific regression passed')


if __name__=='__main__':main()
