"""Compatibility import for the functional experiment package."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from berthing_planner.experiment.chart_geometry import *
