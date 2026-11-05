"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.chart import *
from berthing_planner.experiment.chart import run as _run

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/open_chart_long_beach_stage_a.json')
    parser.add_argument('--archives-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--gdal-bin', type=Path)
    parser.add_argument('--geos-lib', type=Path)
    args = parser.parse_args()
    return _run(config=args.config, archives_dir=args.archives_dir, output_dir=args.output_dir, gdal_bin=args.gdal_bin, geos_lib=args.geos_lib)


if __name__=="__main__":main()
