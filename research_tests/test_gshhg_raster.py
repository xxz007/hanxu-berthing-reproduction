from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from build_gshhg_maps import rasterize


class CoastRasterTests(unittest.TestCase):
    def test_rectangle_and_subcell_island_are_blocked(self):
        polygon = {'level': 1, 'points_EN_m': [(2.2, 2.2), (5.8, 2.2), (5.8, 5.8), (2.2, 5.8)]}
        land, buffered = rasterize([polygon], (0, 0, 8, 8), 8, 0)
        self.assertEqual(int(land.sum()), 16)
        self.assertTrue((land == buffered).all())
        thin = {'level': 1, 'points_EN_m': [(3.05, 2.2), (3.15, 2.2), (3.15, 5.8), (3.05, 5.8)]}
        land, _ = rasterize([thin], (0, 0, 8, 8), 8, 0)
        self.assertEqual(int(land.sum()), 4)
        self.assertTrue(land[2:6, 3].all())

    def test_closed_boundary_and_lake_hierarchy(self):
        outer = {'level': 1, 'points_EN_m': [(1, 1), (31, 1), (31, 31), (1, 31)]}
        lake = {'level': 2, 'points_EN_m': [(8.2, 8.2), (23.8, 8.2), (23.8, 23.8), (8.2, 23.8)]}
        land, _ = rasterize([lake, outer], (0, 0, 32, 32), 32, 0)
        self.assertFalse(land[16, 16])
        self.assertTrue(land[8, 16])
        self.assertTrue(land[0, 16])  # Cell touches the closed outer shoreline at y=1.

    def test_margin_never_clears_land_and_uses_rectangle_distance(self):
        island = {'level': 1, 'points_EN_m': [(10.2, 10.2), (10.8, 10.2), (10.8, 10.8), (10.2, 10.8)]}
        land, buffered = rasterize([island], (0, 0, 32, 32), 32, .5)
        self.assertTrue(buffered[land].all())
        self.assertEqual(int(buffered.sum()), 9)
        self.assertFalse(buffered[10, 12])
        self.assertTrue(buffered[9, 9])


if __name__ == '__main__':
    unittest.main()
