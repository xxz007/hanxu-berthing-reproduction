import math
import random
import unittest

from berthing_planner.quadtree import QuadTree


class QuadTreeTests(unittest.TestCase):
    def setUp(self):
        self.tree = QuadTree((0, 0, 8, 8), [(3, 3, 4, 4)], 3)

    def test_expected_leaf_partition(self):
        expected = {
            (0, 0, 2, 2), (2, 0, 4, 2), (0, 2, 2, 4),
            (2, 2, 3, 3), (3, 2, 4, 3), (2, 3, 3, 4), (3, 3, 4, 4),
            (4, 0, 8, 4), (0, 4, 4, 8), (4, 4, 8, 8),
        }
        leaves = list(self.tree.leaves.values())
        self.assertEqual({leaf.bounds for leaf in leaves}, expected)
        self.assertEqual(sum(leaf.blocked for leaf in leaves), 1)
        self.assertEqual(sum((c-a)*(d-b) for a,b,c,d in expected), 64)
        for index, first in enumerate(leaves):
            for second in leaves[index+1:]:
                a,b,c,d = first.bounds
                l,s,r,t = second.bounds
                self.assertFalse(max(a,l) < min(c,r) and max(b,s) < min(d,t))

    def test_all_base_cell_centers_match_input_occupancy(self):
        for x in range(8):
            for y in range(8):
                with self.subTest(x=x, y=y):
                    self.assertEqual(self.tree.locate((x+.5,y+.5)).blocked, (x,y)==(3,3))

    def test_lookup_examples_and_split_boundary(self):
        for point, box in [
            ((3.5,3.5),(3,3,4,4)), ((6,2),(4,0,8,4)),
            ((2.5,2.5),(2,2,3,3)), ((4,2),(4,0,8,4)),
            ((0,0),(0,0,2,2)), ((2,2),(2,2,3,3)),
        ]:
            with self.subTest(point=point):
                self.assertEqual(self.tree.locate(point).bounds, box)

    def test_outside_and_nonfinite_points(self):
        for point in [(8,2),(2,8),(-.1,2),(math.nan,1),(1,math.inf)]:
            with self.subTest(point=point), self.assertRaises(ValueError):
                self.tree.locate(point)

    def test_large_leaf_has_multiple_small_neighbors(self):
        neighbors = self.tree.neighbors(self.tree.locate((6,2)).code)
        self.assertEqual([cell.center for cell in neighbors], [(3,1),(3.5,2.5),(6,6)])

    def test_adjacency_symmetric_free_and_no_corner_links(self):
        for node in self.tree.leaves.values():
            for neighbor in self.tree.neighbors(node.code):
                self.assertFalse(neighbor.blocked)
                self.assertIn(node, self.tree.neighbors(neighbor.code))
        bottom_left = self.tree.locate((1,1))
        self.assertNotIn(self.tree.locate((2.5,2.5)), self.tree.neighbors(bottom_left.code))
        self.assertEqual(self.tree.neighbors(self.tree.locate((3.5,3.5)).code), [])

    def test_los_known_geometric_cases(self):
        cases = [
            ((1,1),(6,2),True), ((1,1),(6,6),False),
            ((.5,3.5),(7.5,3.5),False), ((.5,3),(7.5,3),False),
            ((3,.5),(3,7.5),False), ((2,4),(4,2),False),
            ((4,4),(6,6),False), ((1,1),(1,1),True),
            ((3.5,3.5),(3.5,3.5),False), ((2.9,3.5),(3.1,3.5),False),
            ((.5,2.999),(7.5,2.999),True),
        ]
        for start, end, expected in cases:
            with self.subTest(start=start,end=end):
                self.assertEqual(self.tree.line_of_sight(start,end), expected)
                self.assertEqual(self.tree.line_of_sight(end,start), expected)

    def test_lookup_ownership_is_not_geometric_collision(self):
        self.assertFalse(self.tree.locate((4,3.5)).blocked)
        self.assertFalse(self.tree.line_of_sight((4,3.5),(6,3.5)))

    def test_depth_limit_conservatively_blocks_mixed_region(self):
        coarse = QuadTree((0,0,8,8), [(3,3,4,4)], 1)
        self.assertEqual(len(coarse.leaves), 4)
        self.assertTrue(coarse.locate((1,1)).blocked)
        self.assertFalse(coarse.line_of_sight((1,1),(2,1)))

    def test_empty_map_and_depth_zero(self):
        empty = QuadTree((0,0,8,8), [], 3)
        self.assertEqual(len(empty.leaves), 1)
        self.assertTrue(empty.line_of_sight((0,0),(7,7)))
        self.assertEqual(empty.neighbors('r'), [])
        full = QuadTree((0,0,8,8), [(3,3,4,4)], 0)
        self.assertTrue(full.root.blocked)
        self.assertEqual(len(full.leaves), 1)

    def test_rectangular_map_and_multiple_obstacles(self):
        tree = QuadTree((0,0,8,4), [(0,0,1,1),(7,3,8,4)], 3)
        for x in range(8):
            for y in range(8):
                point = (x+.5,y*.5+.25)
                expected = (point[0]<1 and point[1]<1) or (point[0]>7 and point[1]>3)
                self.assertEqual(tree.locate(point).blocked, expected)

    def test_generated_maps_against_independent_unit_grid(self):
        rng = random.Random(20261029)
        for _ in range(8):
            occupied = set(rng.sample([(x,y) for x in range(8) for y in range(8)], 12))
            tree = QuadTree((0,0,8,8), [(x,y,x+1,y+1) for x,y in occupied], 3)
            for x in range(8):
                for y in range(8):
                    self.assertEqual(tree.locate((x+.5,y+.5)).blocked, (x,y) in occupied)

    def test_invalid_map_configuration(self):
        for bounds, obstacles, depth in [
            ((0,0,0,8),[],3), ((0,0,8,math.inf),[],3),
            ((0,0,8),[],3), ((0,0,8,8),[(0,0,9,1)],3),
            ((0,0,8,8),[(3,3,2,4)],3), ((0,0,8,8),[],-1),
            ((0,0,8,8),[],True), ((0,0,8,8),[],1.5),
        ]:
            with self.subTest(bounds=bounds,depth=depth), self.assertRaises(ValueError):
                QuadTree(bounds,obstacles,depth)

    def test_los_rejects_points_outside_map(self):
        with self.assertRaises(ValueError):
            self.tree.line_of_sight((1,1),(8,8))


if __name__ == '__main__':
    unittest.main()
