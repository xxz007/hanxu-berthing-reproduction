import math
import unittest

from berthing_planner.occupancy import OccupancyQuadTree
from berthing_planner.replanning import replan_path


class ReplanningTests(unittest.TestCase):
    def test_small_new_obstacle_replaces_local_span(self):
        rows = [[0] * 16 for _ in range(16)]
        rows[7][7] = 1
        tree = OccupancyQuadTree((0, 0, 16, 16), rows)
        original = [(1, 7.5), (4, 7.5), (10, 7.5), (15, 7.5)]
        result = replan_path(tree, original, sample_step_m=.25, anchor_padding_m=.5)
        self.assertTrue(result['changed'])
        self.assertEqual(result['blocked_segment_indices'], [1])
        self.assertEqual(result['path'][:2], original[:2])
        self.assertEqual(result['path'][-2:], original[-2:])
        self.assertEqual(result['replacement_search']['path'][0], result['replacement_start'])
        self.assertEqual(result['replacement_search']['path'][-1], result['replacement_end'])
        self.assertNotEqual(result['replacement_start'], original[0])
        self.assertNotEqual(result['replacement_end'], original[-1])
        self.assertTrue(all(tree.line_of_sight(a, b)
                            for a, b in zip(result['path'], result['path'][1:])))
        self.assertGreater(result['length_m'], result['original_length_m'])
        self.assertEqual(original, [(1, 7.5), (4, 7.5), (10, 7.5), (15, 7.5)])

    def test_obstacle_between_widely_spaced_samples_is_not_skipped(self):
        rows = [[0] * 16 for _ in range(16)]
        rows[7][7] = 1
        tree = OccupancyQuadTree((0, 0, 160, 160), rows)
        result = replan_path(tree, [(10, 75), (150, 75)], sample_step_m=100)
        self.assertTrue(result['changed'])
        self.assertTrue(all(tree.line_of_sight(a, b)
                            for a, b in zip(result['path'], result['path'][1:])))

    def test_separate_obstacles_one_splice_and_no_global_endpoint_move(self):
        rows = [[0] * 16 for _ in range(16)]
        rows[7][5] = rows[7][10] = 1
        tree = OccupancyQuadTree((0, 0, 16, 16), rows)
        original = [(1, 7.5), (4, 7.5), (8, 7.5), (12, 7.5), (15, 7.5)]
        result = replan_path(tree, original, sample_step_m=.3)
        self.assertEqual(result['blocked_segment_indices'], [1, 2])
        self.assertEqual(result['path'][0], original[0])
        self.assertEqual(result['path'][-1], original[-1])
        self.assertEqual(result['path'][:len(result['preserved_prefix'])], result['preserved_prefix'])
        self.assertEqual(result['path'][-len(result['preserved_suffix']):], result['preserved_suffix'])

    def test_clear_and_same_endpoint_paths_are_unchanged(self):
        tree = OccupancyQuadTree((0, 0, 8, 8), [[0] * 8 for _ in range(8)])
        for path in ([(1, 1), (2, 3), (6, 6)], [(1, 1), (1, 1)]):
            result = replan_path(tree, path)
            self.assertFalse(result['changed'])
            self.assertEqual(result['path'], path)

    def test_no_passage_does_not_relax_land(self):
        rows = [[int(i == 3) for i in range(8)] for _ in range(8)]
        tree = OccupancyQuadTree((0, 0, 8, 8), rows)
        with self.assertRaisesRegex(ValueError, 'No path'):
            replan_path(tree, [(1, 4.5), (6, 4.5)], sample_step_m=.5)
        self.assertTrue(tree.locate((3.5, 4.5)).blocked)

    def test_blocked_and_boundary_endpoints_are_rejected(self):
        rows = [[0] * 4 for _ in range(4)]; rows[1][1] = 1
        tree = OccupancyQuadTree((0, 0, 4, 4), rows)
        for point in ((1.5, 1.5), (1, 1)):
            with self.assertRaisesRegex(ValueError, 'endpoints'):
                replan_path(tree, [point, (3, 3)])
        with self.assertRaises(ValueError):
            replan_path(tree, [(1, 3), (4, 3)])

    def test_bad_inputs_are_rejected(self):
        tree = OccupancyQuadTree((0, 0, 4, 4), [[0] * 4 for _ in range(4)])
        for step in (0, -1, math.inf):
            with self.assertRaises(ValueError):
                replan_path(tree, [(1, 1), (3, 3)], sample_step_m=step)
        with self.assertRaises(ValueError):
            replan_path(tree, [(1, 1)])
        with self.assertRaises(ValueError):
            replan_path(tree, [(1, 1), (3, 3)], anchor_padding_m=-1)


if __name__ == '__main__':
    unittest.main()
