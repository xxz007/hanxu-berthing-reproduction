import math
import unittest

from berthing_planner.curves import collision_free, curvature_maximum, wrap
from berthing_planner.g2_bisection import bisect_corners
from berthing_planner.g2_merge import merge_adjacent_turns, subdivision_budget


class G2MergeTests(unittest.TestCase):
    def test_subdivision_two_matches_published_bisection_geometry(self):
        path = [(-1000, 0), (0, 0), (0, 1000)]
        generated = bisect_corners(path, .005)
        budget = subdivision_budget(path, .005, 2)
        self.assertAlmostEqual(budget['corners'][0]['required_trim_m'],
                               generated['corners'][0]['required_trim_m'], places=10)
        self.assertAlmostEqual(subdivision_budget(path, .005)['corners'][0]['required_trim_m'],
                               317.517187211999, delta=.001)

    def test_more_bisection_has_a_positive_budget_limit(self):
        path = [(-1000, 0), (0, 0), (0, 1000)]
        trims = [subdivision_budget(path, .005, n)['corners'][0]['required_trim_m']
                 for n in (1, 2, 4, 8, 64, 1024)]
        limit = subdivision_budget(path, .005, None)['corners'][0]['required_trim_m']
        self.assertAlmostEqual(limit, 224.518557083519, delta=.001)
        self.assertTrue(all(a > b > limit for a, b in zip(trims, trims[1:])))
        self.assertLess(trims[-1]-limit, .001)

    def test_same_sign_pair_merges_at_exact_exterior_intersection(self):
        path = [(-1000, 0), (0, 0), (100, 100), (100, 1000)]
        original = [p for p in path]
        result = merge_adjacent_turns(path, .005, [], (-2000, -2000, 2000, 2000))
        self.assertEqual(result['status'], 'engineering_geometry_passed')
        self.assertFalse(result['author_equivalence'])
        self.assertEqual(result['merges'][0]['original_indices'], [1, 2])
        self.assertEqual(result['merged_virtual_path'], [(-1000, 0), (100, 0), (100, 1000)])
        self.assertEqual(path, original)
        self.assertEqual(result['original_index_groups'], [[0], [1, 2], [3]])
        self.assertLess(result['checks']['endpoint_heading_error_rad'], 1e-10)

    def test_merged_geometry_curvature_g2_and_reflection(self):
        for sign in (-1, 1):
            result = merge_adjacent_turns([(-1000, 0), (0, 0), (100, sign*100),
                                           (100, sign*1000)], .005, [],
                                          (-2000, -2000, 2000, 2000))
            self.assertEqual(result['status'], 'engineering_geometry_passed')
            for p in result['pieces']:
                self.assertLessEqual(curvature_maximum(p), .005+1e-10)
            for p, q in zip(result['pieces'], result['pieces'][1:]):
                a, b = p.evaluate(1), q.evaluate(0)
                self.assertLess(math.dist(a[:2], b[:2]), 1e-7)
                self.assertLess(abs(wrap(a[2]-b[2])), 1e-9)
                self.assertLess(abs(a[3]-b[3]), 1e-10)

    def test_opposite_turns_are_not_replaced_by_same_sign_rule(self):
        path = [(-1000, 0), (0, 0), (100, 100), (1000, 100)]
        result = merge_adjacent_turns(path, .005, [], (-2000, -2000, 2000, 2000))
        self.assertEqual(result['status'], 'no_valid_engineering_merge')
        self.assertFalse(result['pieces'])
        self.assertFalse(result['merges'])

    def test_collision_remains_a_failure_after_merge(self):
        path = [(-1000, 0), (0, 0), (100, 100), (100, 1000)]
        result = merge_adjacent_turns(path, .005, [(-1500, -1500, 1500, 1500)],
                                     (-2000, -2000, 2000, 2000))
        self.assertEqual(result['status'], 'no_valid_engineering_merge')
        self.assertFalse(result['pieces'])
        self.assertTrue(result['rejected_pieces'])
        self.assertTrue(any(not a.get('collision_free', True) for a in result['attempts']))

    def test_straight_route_requires_no_merge(self):
        result = merge_adjacent_turns([(0, 0), (10, 0), (20, 0)], .005, [], (-1, -1, 30, 1))
        self.assertEqual(result['status'], 'engineering_geometry_passed')
        self.assertFalse(result['merges'])
        self.assertTrue(collision_free(result['pieces'], [], (-1, -1, 30, 1)))

    def test_scaling_preserves_merge_and_budget(self):
        p = [(-1000, 0), (0, 0), (100, 100), (100, 1000)]
        a = merge_adjacent_turns(p, .005, [], (-2000, -2000, 2000, 2000))
        b = merge_adjacent_turns([(2*x, 2*y) for x, y in p], .0025, [], (-4000, -4000, 4000, 4000))
        self.assertEqual(a['status'], b['status'])
        self.assertEqual(b['merged_virtual_path'], [(2*x, 2*y) for x, y in a['merged_virtual_path']])
        self.assertAlmostEqual(b['checks']['maximum_actual_curvature_per_m'],
                               a['checks']['maximum_actual_curvature_per_m']/2)

    def test_invalid_input_and_reversal_rejected(self):
        for path, k, parts in (([], .005, 1), ([(0, 0), (0, 0)], .005, 1),
                               ([(0, 0), (1, 0)], 0, 1), ([(0, 0), (1, 0)], .005, 0),
                               ([(0, 0), (1, 0)], .005, 1.5),
                               ([(0, 0), (1, 0), (0, 0)], .005, 2)):
            with self.assertRaises(ValueError):
                subdivision_budget(path, k, parts)


if __name__ == '__main__':
    unittest.main()
