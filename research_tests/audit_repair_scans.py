"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.validation.scanning import *
from berthing_planner.validation.scanning import run as _run

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('repair-dir','run-dir','route-audit-json','prepared-dir','json-out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    return _run(scans_dir=args.repair_dir, routes_dir=args.run_dir, route_audit=args.route_audit_json, prepared_dir=args.prepared_dir, json_out=args.json_out)


if __name__=="__main__":main()
