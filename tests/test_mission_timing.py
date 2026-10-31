import copy
import math
import unittest

from berthing_planner.mission_timing import (
    pairwise_distance_extrema, plan_departure_delays, position_at,
    shift_timeline, time_parameterize)


def moving(points, speed=1, delay=0, scanning=False):
    return time_parameterize([{'points': points, 'kind': 'sweep' if scanning else 'transit',
                               'scanning': scanning}], speed, delay)


def stationary(point, start=0, finish=0):
    return {'segments': [] if start == finish else [
                {'t0_s': start, 't1_s': finish, 'p0': list(point), 'p1': list(point),
                 'scanning': False, 'kind': 'hold'}],
            'start_time_s': start, 'finish_time_s': finish, 'speed_m_s': 0,
            'initial_position': list(point), 'final_position': list(point), 'length_m': 0}


class MissionTimingTests(unittest.TestCase):
    def test_arc_length_clock_and_task_metadata(self):
        legs = [{'points': [[0, 0], [0, 0], [3, 4], [6, 8]], 'scanning': True,
                 'kind': 'sweep', 'task_id': 'scan-1'},
                {'points': [[6, 8], [6, 10]], 'scanning': False, 'kind': 'transit'}]
        original = copy.deepcopy(legs)
        timeline = time_parameterize(legs, 2, start_delay_s=2)
        self.assertEqual(timeline['length_m'], 12)
        self.assertEqual(timeline['start_time_s'], 2)
        self.assertEqual(timeline['finish_time_s'], 8)
        self.assertEqual([(s['t0_s'], s['t1_s']) for s in timeline['segments']],
                         [(2, 4.5), (4.5, 7), (7, 8)])
        self.assertEqual([s['task_id'] for s in timeline['segments'][:2]], ['scan-1', 'scan-1'])
        self.assertNotIn('task_id', timeline['segments'][2])
        self.assertFalse(timeline['segments'][2]['scanning'])
        self.assertEqual(legs, original)

    def test_positions_before_between_boundaries_and_after(self):
        timeline = moving([[0, 0], [3, 4], [3, 6]], speed=2, delay=2)
        for time, expected in ((-10, [0, 0]), (0, [0, 0]), (2, [0, 0]),
                               (3.25, [1.5, 2]), (4.5, [3, 4]), (5, [3, 5]),
                               (5.5, [3, 6]), (1000, [3, 6])):
            self.assertEqual(position_at(timeline, time), expected)

    def test_crossing_between_sample_times_is_found_analytically(self):
        a = moving([[0, 0], [10, 0]])
        b = stationary([4.5, 1], finish=10)
        extrema = pairwise_distance_extrema(a, b)
        self.assertAlmostEqual(extrema['minimum_distance_m'], 1)
        self.assertAlmostEqual(extrema['minimum_time_s'], 4.5)
        self.assertAlmostEqual(extrema['maximum_distance_m'], math.hypot(5.5, 1))
        self.assertEqual(extrema['maximum_time_s'], 10)
        sampled_min = min(math.dist(position_at(a, t), position_at(b, t)) for t in (0, 5, 10))
        self.assertGreater(sampled_min, extrema['minimum_distance_m'])

    def test_two_moving_platforms_cross_at_an_interior_time(self):
        a, b = moving([[-5, 0], [5, 0]]), moving([[0, -5], [0, 5]])
        extrema = pairwise_distance_extrema(a, b)
        self.assertEqual(extrema['minimum_distance_m'], 0)
        self.assertEqual(extrema['minimum_time_s'], 5)
        self.assertAlmostEqual(extrema['maximum_distance_m'], math.sqrt(50))
        self.assertEqual(extrema['maximum_time_s'], 0)

    def test_parallel_motion_and_stationary_platforms(self):
        a, b = moving([[0, 0], [10, 0]]), moving([[0, 25], [10, 25]])
        extrema = pairwise_distance_extrema(a, b, end_time_s=100)
        self.assertEqual(extrema['minimum_distance_m'], 25)
        self.assertEqual(extrema['maximum_distance_m'], 25)
        self.assertEqual(extrema['minimum_time_s'], 0)
        extrema = pairwise_distance_extrema(stationary([0, 0]), stationary([3, 4], 10, 20))
        self.assertEqual(extrema['minimum_distance_m'], 5)
        self.assertEqual(extrema['maximum_distance_m'], 5)
        self.assertEqual(extrema['end_time_s'], 20)

    def test_holds_before_departure_and_after_finish_are_included(self):
        a = moving([[-10, 0], [10, 0]])
        delayed = moving([[0, 0], [0, 10]], delay=20)
        extrema = pairwise_distance_extrema(a, delayed)
        self.assertEqual(extrema['minimum_distance_m'], 0)
        self.assertEqual(extrema['minimum_time_s'], 10)
        finished = moving([[-10, 0], [0, 0]])
        later = moving([[0, -5], [0, 5]], delay=15)
        extrema = pairwise_distance_extrema(finished, later)
        self.assertEqual(extrema['minimum_distance_m'], 0)
        self.assertEqual(extrema['minimum_time_s'], 20)

    def test_nonoverlapping_raw_time_spans_keep_stationary_holds(self):
        a = moving([[0, 0], [10, 0]])
        b = moving([[10, 4], [20, 4]], delay=30)
        extrema = pairwise_distance_extrema(a, b)
        self.assertEqual(extrema['minimum_distance_m'], 4)
        self.assertEqual(extrema['minimum_time_s'], 10)
        self.assertAlmostEqual(extrema['maximum_distance_m'], math.sqrt(116))
        self.assertEqual(extrema['maximum_time_s'], 0)

    def test_explicit_gaps_are_holds_and_maximum_uses_internal_boundaries(self):
        timeline = {'segments': [
            {'t0_s': 0, 't1_s': 10, 'p0': [0, 0], 'p1': [10, 0], 'scanning': False, 'kind': 'support_transit'},
            {'t0_s': 20, 't1_s': 30, 'p0': [10, 0], 'p1': [0, 0], 'scanning': False, 'kind': 'support_transit'}],
            'start_time_s': 0, 'finish_time_s': 30, 'initial_position': [0, 0],
            'final_position': [0, 0], 'speed_m_s': 1, 'length_m': 20}
        self.assertEqual(position_at(timeline, 15), [10, 0])
        extrema = pairwise_distance_extrema(timeline, stationary([0, 1]))
        self.assertEqual(extrema['minimum_distance_m'], 1)
        self.assertAlmostEqual(extrema['maximum_distance_m'], math.sqrt(101))
        self.assertEqual(extrema['maximum_time_s'], 10)

    def test_explicit_window_and_zero_duration_query(self):
        a, b = moving([[0, 0], [10, 0]]), stationary([9, 0])
        part = pairwise_distance_extrema(a, b, start_time_s=1, end_time_s=5)
        self.assertEqual(part['minimum_distance_m'], 4)
        self.assertEqual(part['minimum_time_s'], 5)
        self.assertEqual(part['maximum_distance_m'], 8)
        self.assertEqual(part['maximum_time_s'], 1)
        instant = pairwise_distance_extrema(a, b, start_time_s=30)
        self.assertEqual(instant['start_time_s'], instant['end_time_s'])
        self.assertEqual(instant['minimum_distance_m'], 1)
        self.assertEqual(instant['maximum_distance_m'], 1)
        self.assertEqual(instant['minimum_time_s'], 30)

    def test_shift_preserves_geometry_and_metadata_without_mutation(self):
        timeline = time_parameterize([{'points': [[0, 0], [3, 4]], 'scanning': True,
                                      'kind': 'sweep', 'task_id': 12}], 1, 2)
        original = copy.deepcopy(timeline)
        shifted = shift_timeline(timeline, 5)
        self.assertEqual(shifted['start_time_s'], 7)
        self.assertEqual(shifted['finish_time_s'], 12)
        self.assertEqual(shifted['segments'][0]['task_id'], 12)
        self.assertEqual(shifted['segments'][0]['p1'], [3, 4])
        self.assertEqual(timeline, original)

    def test_priority_delays_resolve_crossing_with_full_horizon_proof(self):
        a, b = moving([[-5, 0], [5, 0]]), moving([[0, -5], [0, 5]])
        result = plan_departure_delays([a, b], 2, delay_step_s=1, max_delay_s=4)
        self.assertEqual(result['delays_s'], [0, 3])
        self.assertEqual(result['mission_end_time_s'], 13)
        check = result['checks'][0]
        self.assertAlmostEqual(check['minimum_distance_m'], 3/math.sqrt(2))
        self.assertAlmostEqual(check['minimum_time_s'], 6.5)
        self.assertEqual(check['end_time_s'], 13)
        self.assertEqual(b['start_time_s'], 0)

    def test_exact_delay_limit_is_checked_after_regular_steps(self):
        a, b = moving([[-5, 0], [5, 0]]), moving([[0, -5], [0, 5]])
        result = plan_departure_delays([a, b], 2, delay_step_s=2, max_delay_s=3)
        self.assertEqual(result['delays_s'], [0, 3])

    def test_no_feasible_plan_does_not_ignore_final_or_initial_holds(self):
        a, b = moving([[-5, 0], [5, 0]]), moving([[0, -5], [0, 5]])
        with self.assertRaisesRegex(ValueError, 'No feasible departure delay'):
            plan_departure_delays([a, b], 3, delay_step_s=1, max_delay_s=2)
        a, b = moving([[-10, 0], [0, 0]]), moving([[0, -5], [0, 5]], delay=15)
        with self.assertRaisesRegex(ValueError, 'No feasible departure delay'):
            plan_departure_delays([a, b], 1, delay_step_s=5, max_delay_s=10)
        with self.assertRaisesRegex(ValueError, 'Initial positions'):
            plan_departure_delays([stationary([0, 0]), moving([[.5, 0], [10, 0]])], 1)

    def test_zero_active_scan_requires_dwell_rule(self):
        with self.assertRaisesRegex(ValueError, 'dwell rule'):
            moving([[1, 1], [1, 1]], scanning=True)
        hold = moving([[1, 1], [1, 1]], delay=5)
        self.assertEqual(hold['segments'], [])
        self.assertEqual(position_at(hold, 100), [1, 1])

    def test_invalid_speed_delay_coordinates_and_leg_gaps_are_rejected(self):
        for speed in (0, -1, math.nan, math.inf, True):
            with self.assertRaises(ValueError):
                moving([[0, 0], [1, 0]], speed=speed)
        with self.assertRaises(ValueError):
            moving([[0, 0], [1, 0]], delay=-1)
        with self.assertRaises(ValueError):
            moving([[math.nan, 0], [1, 0]])
        with self.assertRaisesRegex(ValueError, 'teleportation'):
            time_parameterize([{'points': [[0, 0], [1, 0]], 'scanning': False, 'kind': 'transit'},
                               {'points': [[2, 0], [3, 0]], 'scanning': False, 'kind': 'transit'}], 1)

    def test_tampered_timeline_fields_are_rejected(self):
        valid = moving([[0, 0], [1, 0], [2, 0]])
        mutations = [lambda t: t.update(length_m=3),
                     lambda t: t.update(final_position=[3, 0]),
                     lambda t: t.update(finish_time_s=3),
                     lambda t: t['segments'][1].update(p0=[3, 0]),
                     lambda t: t['segments'][1].update(t0_s=.5),
                     lambda t: t['segments'][0].update(t1_s=0),
                     lambda t: t['segments'][0].update(scanning='false'),
                     lambda t: t['segments'][0].update(p1=[math.inf, 0]),
                     lambda t: t.update(speed_m_s=.5)]
        for mutate in mutations:
            timeline = copy.deepcopy(valid); mutate(timeline)
            with self.assertRaises(ValueError):
                position_at(timeline, 1)
            with self.assertRaises(ValueError):
                pairwise_distance_extrema(timeline, valid)
        with self.assertRaises(ValueError):
            position_at(valid, math.nan)
        with self.assertRaises(ValueError):
            pairwise_distance_extrema(valid, valid, start_time_s=2, end_time_s=1)
        with self.assertRaises(ValueError):
            shift_timeline(valid, -1)
        for params in ({'minimum_separation_m': -1},
                       {'minimum_separation_m': 1, 'delay_step_s': 0},
                       {'minimum_separation_m': 1, 'max_delay_s': math.inf}):
            with self.assertRaises(ValueError):
                plan_departure_delays([valid], **params)


if __name__ == '__main__':
    unittest.main()
