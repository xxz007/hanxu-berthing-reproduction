"""Reference CLI adapter; the algorithm lives in the functional package."""
import argparse
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from berthing_planner.experiment.grid import *
from berthing_planner.experiment.grid import run as _run

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--n', type=int, default=GRID_N,
                        help='Power-of-two grid size; default 1024 gives 5m cells')
    parser.add_argument('--gdal-bin', type=Path)
    parser.add_argument('--geos-lib', type=Path)
    args = parser.parse_args()
    return _run(prepared_dir=args.prepared_dir, output_dir=args.output_dir, n=args.n, gdal_bin=args.gdal_bin, geos_lib=args.geos_lib)


if __name__=="__main__":main()
