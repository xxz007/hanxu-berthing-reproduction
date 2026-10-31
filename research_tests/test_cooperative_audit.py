"""Tamper and between-snapshot checks for the independent A3 auditor."""
from copy import deepcopy
import math
import unittest

import numpy as np

from audit_cooperative_coverage import (
    audit_data, continuous_pair_extrema, task_accounting, timeline_checks)


def segment(t0, t1, start, end, *, kind='transit', scanning=False, task_id=None):
    return {'t0_s': t0, 't1_s': t1, 'p0': start, 'p1': end,
            'kind': kind, 'scanning': scanning, 'task_id': task_id}


def fixture():
    duration = 100/1.5
    platforms = {}
    tasks = []
    for i, y in enumerate((20, 80)):
        name, task_id = 'uuv_'+str(i+1), 'scan_00'+str(i)
        points = [[20, y], [120, y]]
        kind = 'sweep' if i == 0 else 'supplement'
        leg = {'kind': kind, 'scanning': True, 'points': points, 'length_m': 100, 'task_id': task_id}
        platforms[name] = {'role': 'coverage', 'path': points, 'legs': [leg], 'length_m': 100,
                           'task_ids': [task_id],
                           'timeline': [segment(0, duration, *points, kind=kind, scanning=True, task_id=task_id)]}
        tasks.append({'task_id': task_id, 'source_leg_index': 0 if i == 0 else 2,
                      'kind': kind, 'points': deepcopy(points), 'length_m': 100})
    support = [[20, 140], [120, 140]]
    platforms['usv'] = {'role': 'surface_support', 'path': support,
                        'legs': [{'kind': 'support_transit', 'scanning': False,
                                  'points': support, 'length_m': 100}], 'length_m': 100,
                        'timeline': [segment(0, 100/3, *support, kind='support_transit'),
                                     segment(100/3, duration, support[-1], support[-1], kind='hold')]}
    # A2 metrics are a fixed trusted input here; these tests audit inheritance,
    # not a new footprint-area calculation on the synthetic geometry.
    coverage = {'target_area_m2': 10000, 'covered_area_m2': 10000, 'missed_area_m2': 0,
                'repeated_area_m2': 0, 'coverage_ratio': 1, 'missed_ratio': 0, 'repeated_ratio': 0}
    result = {'status': 'cooperative_geometry_passed', 'target_geometry_sha256': 'test-target',
              'mission_end_s': duration, 'platforms': platforms, 'tasks': tasks,
              'allocation': {'task_owner': {'scan_000': 'uuv_1', 'scan_001': 'uuv_2'}},
              'coverage': deepcopy(coverage)}
    a2 = {'legs': [deepcopy(platforms['uuv_1']['legs'][0]),
                   {'kind': 'transit', 'scanning': False, 'points': [[120, 20], [20, 80]],
                    'length_m': math.hypot(100, 60)}, deepcopy(platforms['uuv_2']['legs'][0])],
          'vector_coverage': coverage}
    config = {'platform_speeds_mps': {'usv': 3, 'uuv_1': 1.5, 'uuv_2': 1.5},
              'minimum_horizontal_separation_m': 25, 'support_range_m': 3000}
    return result, a2, config, np.zeros((20, 20), dtype=bool), [0, 0, 200, 200]


class CooperativeAuditTests(unittest.TestCase):
    def test_crossing_between_safe_endpoint_snapshots(self):
        a = [segment(0, 10, [0, 0], [10, 0])]
        b = [segment(0, 10, [5, 5], [5, -5])]
        r = continuous_pair_extrema(a, b, 10)
        self.assertAlmostEqual(r['minimum_distance_m'], 0)
        self.assertAlmostEqual(r['minimum_at_s'], 5)
        self.assertAlmostEqual(r['maximum_distance_m'], math.sqrt(50))

    def test_stationary_vs_moving_interior_minimum(self):
        a = [segment(0, 10, [0, 0], [10, 0])]
        b = [segment(0, 10, [3, 4], [3, 4], kind='hold')]
        r = continuous_pair_extrema(a, b, 10)
        self.assertAlmostEqual(r['minimum_distance_m'], 4)
        self.assertAlmostEqual(r['minimum_at_s'], 3)
        self.assertAlmostEqual(r['maximum_distance_m'], math.sqrt(65))
        self.assertAlmostEqual(r['maximum_at_s'], 10)

    def test_all_breakpoints_and_pre_post_holds_are_included(self):
        a = [segment(0, 5, [0, 0], [0, 0], kind='hold'),
             segment(5, 15, [0, 0], [10, 0]),
             segment(15, 20, [10, 0], [10, 0], kind='hold')]
        b = [segment(0, 20, [5, 4], [5, 4], kind='hold')]
        r = continuous_pair_extrema(a, b, 20)
        self.assertAlmostEqual(r['minimum_at_s'], 10)
        self.assertAlmostEqual(r['minimum_distance_m'], 4)
        self.assertAlmostEqual(r['maximum_distance_m'], math.sqrt(41))
        self.assertEqual(r['common_affine_intervals_checked'], 3)

    def test_range_peak_at_a_platform_internal_breakpoint(self):
        a = [segment(0, 5, [0, 0], [20, 0]), segment(5, 10, [20, 0], [0, 0])]
        b = [segment(0, 10, [0, 0], [0, 0], kind='hold')]
        r = continuous_pair_extrema(a, b, 10)
        self.assertAlmostEqual(r['maximum_distance_m'], 20)
        self.assertAlmostEqual(r['maximum_at_s'], 5)

    def test_valid_complete_cooperative_fixture(self):
        r = audit_data(*fixture())
        self.assertTrue(r['passed'])
        self.assertEqual(r['tasks']['A2_active_task_count'], 2)
        self.assertEqual(r['continuous_pair_distances']['uuv_1__uuv_2']['minimum_distance_m'], 60)

    def test_timeline_dictionary_headers_are_checked_against_segments(self):
        args = list(fixture())
        for platform in args[0]['platforms'].values():
            platform['timeline'] = {'segments': platform['timeline'], 'start_time_s': 0,
                                    'finish_time_s': args[0]['mission_end_s']}
        self.assertTrue(audit_data(*args)['passed'])
        args[0]['platforms']['usv']['timeline']['finish_time_s'] -= 1
        r = audit_data(*args)
        self.assertFalse(r['passed'])
        self.assertEqual(r['timeline']['usv']['timeline_header_errors'], ['finish_time_s'])

    def test_reversing_a_task_keeps_its_original_footprint(self):
        result, a2, *_ = fixture()
        result['tasks'][0]['points'].reverse()
        self.assertTrue(task_accounting(result, a2)['passed'])

    def test_missing_and_duplicate_task_are_rejected(self):
        result, a2, *_ = fixture()
        result['tasks'].pop()
        self.assertFalse(task_accounting(result, a2)['passed'])
        result, a2, *_ = fixture()
        result['tasks'][1] = deepcopy(result['tasks'][0])
        r = task_accounting(result, a2)
        self.assertFalse(r['passed'])
        self.assertFalse(r['source_leg_indices_exactly_once'])
        self.assertFalse(r['task_ids_unique'])

    def test_geometry_shift_and_double_assignment_are_rejected(self):
        result, a2, *_ = fixture()
        result['tasks'][0]['points'][-1][0] += 1
        self.assertFalse(task_accounting(result, a2)['passed'])
        result, a2, *_ = fixture()
        result['platforms']['uuv_2']['task_ids'].append('scan_000')
        self.assertFalse(task_accounting(result, a2)['passed'])

    def test_usv_scanning_and_unexecuted_uuv_scan_are_rejected(self):
        args = list(fixture())
        args[0]['platforms']['usv']['timeline'][0]['scanning'] = True
        self.assertFalse(audit_data(*args)['passed'])
        args = list(fixture())
        args[0]['platforms']['uuv_1']['timeline'][0]['scanning'] = False
        self.assertFalse(audit_data(*args)['passed'])

    def test_timestamp_tamper_and_missing_final_hold_are_rejected(self):
        args = list(fixture())
        args[0]['platforms']['uuv_1']['timeline'][0]['t1_s'] -= 1
        r = audit_data(*args)
        self.assertFalse(r['passed'])
        self.assertFalse(r['timeline']['uuv_1']['passed'])
        self.assertFalse(r['continuous_pair_distances'])
        args = list(fixture())
        args[0]['platforms']['usv']['timeline'].pop()
        self.assertFalse(audit_data(*args)['passed'])

    def test_clock_gaps_and_teleporting_holds_are_rejected(self):
        result, *_ = fixture()
        platform = result['platforms']['usv']
        platform['timeline'][1]['t0_s'] += 1
        self.assertFalse(timeline_checks(platform, 3, result['mission_end_s'])['passed'])
        result, *_ = fixture()
        platform = result['platforms']['usv']
        platform['timeline'][1]['p1'] = [130, 140]
        self.assertFalse(timeline_checks(platform, 3, result['mission_end_s'])['passed'])

    def test_support_range_and_raw_cell_collision_are_not_waived(self):
        args = list(fixture())
        args[2]['support_range_m'] = 60
        self.assertFalse(audit_data(*args)['passed'])
        args = list(fixture())
        args[3][2, 5] = True  # The UUV 1 scan touches this closed cell edge.
        r = audit_data(*args)
        self.assertFalse(r['passed'])
        self.assertEqual(r['raw_grid_collision']['uuv_1']['segments_hitting_raw_blocked_cells'], 1)

    def test_coverage_dictionary_cannot_be_relabelled(self):
        args = list(fixture())
        args[0]['coverage']['coverage_ratio'] = .99
        r = audit_data(*args)
        self.assertFalse(r['passed'])
        self.assertFalse(r['coverage_metrics_identical_to_A2'])


if __name__ == '__main__':
    unittest.main()
