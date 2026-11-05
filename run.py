"""Run the final fixed-case experiment without replaying its development history."""
import argparse
from datetime import datetime
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'src'))
from berthing_planner.experiment.workflow import run


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/experiment.json')
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--archives-dir', type=Path, help='Directory containing both fixed SHA-verified ENC ZIPs')
    source.add_argument('--download', action='store_true', help='Fetch official NOAA archives; reject changed snapshots and retain them locally')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'outputs'/datetime.now().strftime('%Y%m%d_%H%M%S_experiment'))
    parser.add_argument('--qa-script-dir', type=Path, help='Optional local figure QA tools; not required to run')
    args=parser.parse_args()
    run(config=args.config, output_dir=args.output_dir, archives_dir=args.archives_dir, qa_script_dir=args.qa_script_dir)


if __name__=='__main__':
    main()
