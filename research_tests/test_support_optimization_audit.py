"""Negative controls for A9 matched-condition improvement claims."""
import copy
import unittest
from audit_support_optimization import comparison_errors


class OptimizationAuditTests(unittest.TestCase):
    def fixture(self):
        cfg = dict.fromkeys(('source_a7_scientific_sha256', 'target_geometry_sha256',
                             'hazards_geometry_sha256', 'navigation_sha256'), 'pin')
        cfg.update(mobile_design_range_m=2300, minimum_separation_m=25,
                   hazard_margin_m=50, usv_radius_m=40, usv_cruise_mps=3,
                   tangential_accel_mps2=.1, lateral_accel_mps2=.05)
        baseline = {'uuv_platforms': {'uuv_1': {'pieces': [1]}}, 'scan_windows': [1, 2],
                    'mission_end_s': 100, 'coverage_lower_ratio_unchanged': .9991,
                    'mobile_usv': {'initial_pose': [0, 0, 0], 'legs': [1, 2]},
                    'strategies': {'scheduled_mobile': {'usv_length_m': 1000, 'usv_motion_duration_s': 500}}}
        result = copy.deepcopy(baseline)
        result.update(stage='A9', mobile_usv={'initial_pose': [0, 0, 0],
                      'legs': [{'profile': {'length_m': 200, 'duration_s': 100}}]},
                      strategies={'scheduled_mobile': {'usv_length_m': 200, 'usv_motion_duration_s': 100,
                      'pairs': {'usv__uuv_1': {'converged': True, 'minimum_distance_lower_m': 30, 'maximum_distance_upper_m': 2299}}}},
                      comparison={'source_a8_length_m': 1000, 'source_a8_transfer_count': 2,
                      'source_a8_motion_duration_s': 500, 'optimized_length_m': 200,
                      'optimized_transfer_count': 1, 'length_saved_m': 800, 'length_saved_ratio': .8,
                      'strict_length_improvement': True, 'all_constraints_certified': True})
        return result, baseline, cfg, copy.deepcopy(cfg)

    def test_matched_feasible_improvement(self):
        self.assertEqual(comparison_errors(*self.fixture()), [])

    def test_false_shorter_route_cannot_hide_range_violation(self):
        result, base, cfg, old = self.fixture()
        result['strategies']['scheduled_mobile']['pairs']['usv__uuv_1']['maximum_distance_upper_m'] = 2301
        self.assertIn('whole mission A9 constraints', comparison_errors(result, base, cfg, old))

    def test_relaxed_range_cannot_be_called_same_condition(self):
        result, base, cfg, old = self.fixture(); cfg['mobile_design_range_m'] = 2400
        self.assertIn('changed A8 constraint: mobile_design_range_m', comparison_errors(result, base, cfg, old))

    def test_deleted_scan_cannot_improve_cost(self):
        result, base, cfg, old = self.fixture(); result['scan_windows'].pop()
        self.assertIn('changed A8 task: scan_windows', comparison_errors(result, base, cfg, old))

    def test_claimed_savings_must_match_geometry(self):
        result, base, cfg, old = self.fixture(); result['comparison']['length_saved_ratio'] = .99
        self.assertIn('comparison totals or improvement claim', comparison_errors(result, base, cfg, old))

    def test_omitted_whole_mission_pair_cannot_pass(self):
        result, base, cfg, old = self.fixture()
        result['strategies']['scheduled_mobile']['pairs'].clear()
        self.assertIn('missing or extra whole mission distance pair', comparison_errors(result, base, cfg, old))


if __name__ == '__main__':
    unittest.main()
