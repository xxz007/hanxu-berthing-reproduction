"""Hand-checkable tests for the independent A2 distance/collision audit."""
import math
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from audit_open_chart_coverage import (
    active_pass_counts, closed_segment_blocked_cells,
    main, point_segment_distance_squared, route_checks)


def leg(points, kind='sweep', scanning=True):
    xy = np.asarray(points, dtype=float)
    return {'kind': kind, 'scanning': scanning, 'points': points,
            'length_m': float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())}


class CoverageAuditTests(unittest.TestCase):
    def test_closed_segment_distance_not_distance_to_infinite_line(self):
        points = np.array([[1, 1], [-1, 0], [3, 0], [1, 0], [2, 1]], dtype=float)
        actual = point_segment_distance_squared(points, [0, 0], [2, 0])
        np.testing.assert_allclose(actual, [1, 1, 1, 0, 1])
        np.testing.assert_allclose(point_segment_distance_squared(points, [2, 0], [0, 0]), actual)

    def test_degenerate_segment_is_a_point(self):
        points = np.array([[1, 2], [4, 6], [1, 3]], dtype=float)
        np.testing.assert_allclose(point_segment_distance_squared(points, [1, 2], [1, 2]), [0, 25, 1])

    def test_two_active_passes_repeat_but_transit_does_not_scan(self):
        legs = [leg([[-2, 0], [2, 0]]), leg([[-2, 2], [2, 2]]),
                leg([[0, -5], [0, 5]], 'transit', False)]
        points = np.array([[0, 1], [0, 0], [0, 3], [4, 0]], dtype=float)
        np.testing.assert_array_equal(active_pass_counts(points, legs, 1), [2, 1, 1, 0])

    def test_intermediate_vertices_do_not_multiply_one_pass(self):
        points = np.array([[0, 0], [.5, 0], [-1.5, 0]], dtype=float)
        legs = [leg([[-2, 0], [0, 0], [2, 0]])]
        np.testing.assert_array_equal(active_pass_counts(points, legs, 1), [1, 1, 1])

    def test_end_caps_and_stationary_scan_are_circular_footprints(self):
        points = np.array([[3, 0], [3.01, 0], [-3, 0], [0, 1]], dtype=float)
        np.testing.assert_array_equal(active_pass_counts(points, [leg([[-2, 0], [2, 0]])], 1), [1, 0, 1, 1])
        points = np.array([[0, 0], [1, 0], [1.001, 0], [.8, .8]], dtype=float)
        np.testing.assert_array_equal(active_pass_counts(points, [leg([[0, 0], [0, 0]])], 1), [1, 1, 0, 0])

    def test_bad_transit_scanning_flag_is_ignored_and_reported(self):
        segments = [leg([[0, 0], [1, 0]], 'transit', True)]
        counts = active_pass_counts(np.array([[.5, 0]]), segments, 1)
        np.testing.assert_array_equal(counts, [0])
        result = route_checks({'path': [[0, 0], [1, 0]], 'legs': segments, 'length_m': 1})
        self.assertEqual(result['transit_legs_with_scanning_flag'], [0])
        self.assertFalse(result['passed'])

    def test_closed_cell_edge_and_corner_are_collision(self):
        blocked = np.zeros((3, 3), dtype=bool)
        blocked[1, 1] = True  # Closed rectangle [1,2] x [1,2].
        bounds = [0, 0, 3, 3]
        for a, b in (([0, 1], [3, 1]), ([0, 2], [2, 0]), ([1, 1], [1, 1]),
                     ([1.5, 0], [1.5, 3]), ([1.5, 1.5], [1.5, 1.5])):
            np.testing.assert_array_equal(closed_segment_blocked_cells(a, b, blocked, bounds), [[1, 1]])
        for a, b in (([0, .999999], [3, .999999]), ([0, 0], [.5, .5]), ([3, 3], [4, 4])):
            self.assertEqual(len(closed_segment_blocked_cells(a, b, blocked, bounds)), 0)

    def test_native_cells_are_not_replaced_by_a_block_bounding_box(self):
        blocked = np.zeros((3, 3), dtype=bool)
        blocked[0, 0] = blocked[2, 2] = True
        hits = closed_segment_blocked_cells([.2, 1.5], [2.8, 1.5], blocked, [0, 0, 3, 3])
        self.assertEqual(len(hits), 0)

    def test_non_unit_grid_origin_and_spacing(self):
        blocked = np.zeros((2, 2), dtype=bool)
        blocked[1, 0] = True  # [10,15] x [25,30].
        np.testing.assert_array_equal(
            closed_segment_blocked_cells([5, 25], [20, 25], blocked, [10, 20, 20, 30]), [[1, 0]])
        self.assertEqual(len(closed_segment_blocked_cells([11, 21], [14, 24], blocked, [10, 20, 20, 30])), 0)

    def test_path_legs_and_lengths_agree_with_shared_endpoint(self):
        legs = [leg([[0, 0], [2, 0]]), leg([[2, 0], [2, 1]], 'transit', False)]
        r = route_checks({'path': [[0, 0], [2, 0], [2, 1]], 'legs': legs, 'length_m': 3})
        self.assertTrue(r['passed'])
        self.assertEqual(r['recomputed_path_length_m'], 3)
        self.assertEqual(r['active_pass_count'], 1)
        self.assertEqual(r['transit_leg_count'], 1)

    def test_consecutive_duplicate_points_are_not_false_discontinuity(self):
        legs = [leg([[0, 0], [1, 0], [1, 0]]), leg([[1, 0], [2, 0]])]
        r = route_checks({'path': [[0, 0], [1, 0], [1, 0], [2, 0]], 'legs': legs, 'length_m': 2})
        self.assertTrue(r['passed'])

    def test_gap_curve_and_length_errors_remain_failures(self):
        bad_gap = [leg([[0, 0], [1, 0]]), leg([[2, 0], [3, 0]])]
        r = route_checks({'path': [[0, 0], [1, 0], [2, 0], [3, 0]], 'legs': bad_gap, 'length_m': 3})
        self.assertFalse(r['passed'])
        self.assertEqual(r['maximum_leg_gap_m'], 1)
        bent = [leg([[0, 0], [1, 1], [2, 0]])]
        r = route_checks({'path': bent[0]['points'], 'legs': bent, 'length_m': 2*math.sqrt(2)})
        self.assertFalse(r['passed'])
        self.assertEqual(r['non_straight_active_leg_indices'], [0])
        mismatch = [leg([[0, 0], [2, 0]])]
        r = route_checks({'path': [[0, 0], [2, 0]], 'legs': mismatch, 'length_m': 1})
        self.assertFalse(r['passed'])

    def test_path_cannot_skip_a_leg_even_with_same_total_length(self):
        legs = [leg([[0, 0], [1, 0], [2, 0]])]
        r = route_checks({'path': [[0, 0], [2, 0]], 'legs': legs, 'length_m': 2})
        self.assertFalse(r['passed'])
        self.assertFalse(r['path_matches_ordered_legs'])

    def test_cli_known_square_coverage_and_target_fingerprint(self):
        # A horizontal scan through the centre of a 20x20 square, radius 10,
        # covers every point of that square. Its exact clipped area is 400.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, prepared, grid = (root/name for name in ('run', 'prepared', 'grid'))
            for p in (run, prepared, grid):
                p.mkdir()
            shape = {'type': 'Polygon', 'coordinates': [[[0, 0], [20, 0], [20, 20], [0, 20], [0, 0]]]}
            target = prepared/'searchable.geojson'
            target.write_text(json.dumps({'type': 'FeatureCollection',
                                         'features': [{'type': 'Feature', 'geometry': shape, 'properties': {}}]}))
            fingerprint = hashlib.sha256(target.read_bytes()).hexdigest()
            (prepared/'manifest.json').write_text(json.dumps(
                {'derived_files_sha256': {'searchable.geojson': fingerprint}}))
            (prepared/'summary.json').write_text(json.dumps(
                {'searchable_area_m2': 400, 'bbox_local_m': [0, 0, 20, 20]}))
            (grid/'grid.json').write_text(json.dumps(
                {'n': 4, 'bounds': [0, 0, 20, 20], 'rows_bottom_to_top': [[0]*4 for _ in range(4)]}))
            (run/'config.json').write_text(json.dumps(
                {'swath_width_m': 20, 'lane_spacing_m': 16, 'lane_offset_m': 8, 'target_coverage_ratio': 1}))
            result = {'path': [[0, 10], [20, 10]], 'legs': [leg([[0, 10], [20, 10]])],
                      'length_m': 20, 'target_geometry_sha256': fingerprint,
                      'vector_coverage': {'target_area_m2': 400, 'covered_area_m2': 400,
                                          'missed_area_m2': 0, 'repeated_area_m2': 0,
                                          'coverage_ratio': 1, 'missed_ratio': 0, 'repeated_ratio': 0}}
            (run/'result.json').write_text(json.dumps(result))
            output = root/'audit.json'
            arguments = ['audit', '--run-dir', str(run), '--prepared-dir', str(prepared),
                         '--grid-dir', str(grid), '--json-out', str(output)]
            with patch.object(sys, 'argv', arguments), redirect_stdout(io.StringIO()):
                main()
            audit = json.loads(output.read_text())
            self.assertTrue(audit['passed'])
            self.assertEqual(audit['candidate_interior_grid_centres'], 4)
            self.assertEqual(audit['sampled_interior_metrics']['coverage_ratio'], 1)
            self.assertEqual(audit['sampled_minus_native_metrics']['covered_area_m2'], 0)
            # Merely reporting the right area cannot conceal a changed target.
            result['target_geometry_sha256'] = 'wrong-target-fingerprint'
            (run/'result.json').write_text(json.dumps(result))
            arguments[-1] = str(root/'audit_wrong_target.json')
            with patch.object(sys, 'argv', arguments), redirect_stdout(io.StringIO()), self.assertRaises(SystemExit):
                main()
            self.assertFalse(json.loads((root/'audit_wrong_target.json').read_text())['target_unchanged'])


if __name__ == '__main__':
    unittest.main()
