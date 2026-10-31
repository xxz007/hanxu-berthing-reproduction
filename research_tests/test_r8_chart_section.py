from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from run_cross_chart_r8 import chart_section


class ChartSectionTests(unittest.TestCase):
    def test_entry_exit_and_original_waypoints_preserved(self):
        path = [(-2, 1), (1, 1), (2, 2), (5, 2)]
        prefix, local, suffix = chart_section(path, (0, 0, 3, 3))
        self.assertEqual(prefix[0], path[0])
        self.assertEqual(suffix[-1], path[-1])
        self.assertAlmostEqual(local[0][0], 1e-6)
        self.assertAlmostEqual(local[-1][0], 3-1e-6)
        self.assertEqual(local[1:-1], path[1:3])
        self.assertEqual(prefix[-1], local[0])
        self.assertEqual(suffix[0], local[-1])

    def test_tangent_only_and_multiple_entries_are_explicit(self):
        with self.assertRaisesRegex(ValueError, 'does not enter'):
            chart_section([(-1, 0), (4, 0)], (0, 0, 3, 3))
        with self.assertRaisesRegex(ValueError, 'one contiguous'):
            chart_section([(-1, 1), (4, 1), (1, 2)], (0, 0, 3, 3))


if __name__ == '__main__':
    unittest.main()
