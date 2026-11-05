"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.contracts import *
from berthing_planner.experiment.contracts import run as _run

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--a2-run-dir', type=Path, required=True)
    parser.add_argument('--reference-config', type=Path,
                        default=ROOT/'configs/cooperative_coverage_a3.json')
    parser.add_argument('--output-config', type=Path, required=True)
    args = parser.parse_args(argv)
    return _run(coverage_dir=args.a2_run_dir, reference_config=args.reference_config, output_config=args.output_config)


if __name__=="__main__":main()
