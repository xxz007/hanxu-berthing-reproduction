import itertools
import math
import random
import unittest

from berthing_planner.occupancy import OccupancyQuadTree
from berthing_planner.raster_map import RasterQuadTree
from berthing_planner.search import astar


def reference(rows, bounds=None):
    n = len(rows)
    return RasterQuadTree({'schema_version': 1,
                          'status': 'reconstructed_centerline_not_author_raw_map',
                          'bounds': bounds or (0, 0, n, n), 'n': n,
                          'rows_bottom_to_top': rows})


class OccupancyTests(unittest.TestCase):
    def test_uniform_regions_and_grid_orientation(self):
        for value in (0, 1):
            tree = OccupancyQuadTree((0, 0, 1024, 512), [[value] * 16 for _ in range(16)])
            self.assertEqual(len(tree.leaves), 1)
            self.assertEqual(tree.root.blocked, bool(value))
        tree = OccupancyQuadTree((0, 0, 4, 4), [[1, 1, 0, 0], [1, 1, 0, 0],
                                                 [0, 0, 0, 0], [0, 0, 0, 0]])
        self.assertEqual(len(tree.leaves), 4)
        self.assertTrue(tree.locate((.5, .5)).blocked)
        self.assertFalse(tree.locate((.5, 3.5)).blocked)
        self.assertEqual(tree.obstacles, ((0.0, 0.0, 2.0, 2.0),))

    def test_all_two_by_two_maps_match_reference_queries(self):
        points = [(x / 2, y / 2) for x in range(4) for y in range(4)]
        for bits in itertools.product((0, 1), repeat=4):
            rows = [bits[:2], bits[2:]]
            tree, old = OccupancyQuadTree((0, 0, 2, 2), rows), reference(rows)
            for a in points:
                self.assertEqual(tree.locate(a).blocked, old.locate(a).blocked)
                for b in points:
                    self.assertEqual(tree.line_of_sight(a, b), old.line_of_sight(a, b), (bits, a, b))
            for code, cell in tree.leaves.items():
                if not cell.blocked:
                    self.assertEqual([c.code for c in tree.neighbors(code)],
                                     [c.code for c in old.neighbors(code)])

    def test_four_by_four_filled_regions_and_random_queries(self):
        rng = random.Random(2903)
        cases = [[[int(i < 2 and j < 2) for i in range(4)] for j in range(4)]]
        cases += [[[rng.randrange(2) for _ in range(4)] for _ in range(4)] for _ in range(64)]
        for rows in cases:
            bounds = (11500000, 120000, 11570000, 160000)
            tree, old = OccupancyQuadTree(bounds, rows), reference(rows, bounds)
            for code, cell in tree.leaves.items():
                if not cell.blocked:
                    self.assertEqual([c.code for c in tree.neighbors(code)],
                                     [c.code for c in old.neighbors(code)])
            for _ in range(80):
                a, b = [(bounds[0] + rng.random() * 70000,
                         bounds[1] + rng.random() * 40000) for _ in range(2)]
                self.assertEqual(tree.line_of_sight(a, b), old.line_of_sight(a, b))

    def test_closed_corner_and_half_open_ownership(self):
        tree = OccupancyQuadTree((0, 0, 4, 4), [[0, 0, 0, 0], [0, 1, 0, 0],
                                                [0, 0, 0, 0], [0, 0, 0, 0]])
        self.assertFalse(tree.line_of_sight((.5, 1.5), (1.5, .5)))
        self.assertFalse(tree.line_of_sight((1, 1), (1, 1)))
        self.assertFalse(tree.locate((2, 2)).blocked)
        self.assertFalse(tree.line_of_sight((2, 2), (2, 2)))
        with self.assertRaises(ValueError):
            tree.locate((4, 2))
        with self.assertRaises(ValueError):
            tree.line_of_sight((0, 0), (math.inf, 0))

    def test_astar_and_dijkstra_same_graph(self):
        rows = [[int(i in (3, 4) and j < 6) for i in range(8)] for j in range(8)]
        tree = OccupancyQuadTree((0, 0, 8, 8), rows)
        for policy in ('local', 'theta_relaxation'):
            result = astar(tree, (1, 1), (6, 1), parent_policy=policy)
            self.assertTrue(all(tree.line_of_sight(a, b)
                                for a, b in zip(result['path'], result['path'][1:])))
        a = astar(tree, (1, 1), (6, 1))
        d = astar(tree, (1, 1), (6, 1), heuristic_weight=0)
        self.assertAlmostEqual(a['length_m'], d['length_m'])

    def test_bad_input_rejected(self):
        for rows in ([], [[0] * 3] * 3, [[0, 0], [0]], [[0, 2], [0, 0]]):
            with self.assertRaises(ValueError):
                OccupancyQuadTree((0, 0, 2, 2), rows)
        for bounds in ((0, 0, 0, 1), (0, 0, 1, math.nan), (0, 0, 1)):
            with self.assertRaises(ValueError):
                OccupancyQuadTree(bounds, [[0]])


if __name__ == '__main__':
    unittest.main()
