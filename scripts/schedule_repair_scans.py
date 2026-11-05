"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.scanning import *
from berthing_planner.experiment.scanning import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run-dir','audit-json','prepared-dir','output-dir'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'configs/coverage_repair_a6.json')
    args=parser.parse_args()
    return _run(routes_dir=args.run_dir, route_audit=args.audit_json, prepared_dir=args.prepared_dir, output_dir=args.output_dir, config=args.config)


if __name__=="__main__":main()
