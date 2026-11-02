import math
import unittest
from berthing_planner.curves import Piece, line
from berthing_planner.motion_feasibility import convex_hull, forward_turn_witnesses, stopping_profile, profile_state


class MotionFeasibilityTests(unittest.TestCase):
    def setUp(self):
        self.hull = convex_hull([(0,0),(100,0),(100,100),(0,100),(20,20)])

    def test_hull_keeps_supporting_outer_vertices(self):
        self.assertEqual(self.hull, [(0,0),(100,0),(100,100),(0,100)])

    def test_normal_outward_required_distance_is_radius(self):
        w = forward_turn_witnesses((99,50), 0, self.hull, 20)[0]
        self.assertAlmostEqual(w['available_outward_distance_m'],1)
        self.assertAlmostEqual(w['unavoidable_outward_distance_m'],20)
        self.assertAlmostEqual(w['necessary_radius_upper_bound_m'],1)

    def test_oblique_heading_matches_circle_integral(self):
        w = forward_turn_witnesses((99,50), math.pi/6, self.hull, 20)[0]
        self.assertAlmostEqual(w['unavoidable_outward_distance_m'],10)

    def test_tangent_heading_has_no_outward_obligation(self):
        self.assertEqual(forward_turn_witnesses((99,50),math.pi/2,self.hull,20),[])

    def test_inward_heading_can_be_clear(self):
        self.assertEqual(forward_turn_witnesses((99,50),math.pi,self.hull,20),[])

    def test_equality_is_not_a_failure(self):
        self.assertEqual(forward_turn_witnesses((80,50),0,self.hull,20),[])

    def test_entry_is_checked_by_reversing_tangent(self):
        self.assertTrue(forward_turn_witnesses((1,50),math.pi,self.hull,20))

    def test_invalid_boundary_parameters_rejected(self):
        for r in (0,-1,math.nan):
            with self.assertRaises(ValueError):forward_turn_witnesses((1,2),0,self.hull,r)
        with self.assertRaises(ValueError):convex_hull([(0,0),(1,0),(2,0)])

    def test_trapezoid_hand_solution(self):
        p=stopping_profile([line((0,0),(100,0))],2,1,1)
        self.assertAlmostEqual(p['duration_s'],52)
        self.assertAlmostEqual(profile_state(p,1)['distance_m'],.5)
        self.assertAlmostEqual(profile_state(p,51)['distance_m'],99.5)

    def test_short_segment_is_triangular(self):
        p=stopping_profile([line((0,0),(1,0))],10,1,1)
        self.assertAlmostEqual(p['duration_s'],2)
        self.assertAlmostEqual(p['segments'][0]['peak_speed_mps'],1)

    def test_arc_lateral_limit_and_zero_speed_joins(self):
        pieces=[line((-100,0),(0,0)),Piece('arc',pose=(0,0,0),length=40*math.pi,curvature=1/40),line((0,80),(-100,80))]
        p=stopping_profile(pieces,1.5,.1,.05)
        self.assertAlmostEqual(p['segments'][1]['peak_speed_mps'],math.sqrt(2))
        self.assertAlmostEqual(p['segments'][1]['maximum_lateral_acceleration_mps2'],.05)
        for s in p['segments']:
            self.assertAlmostEqual(profile_state(p,s['t1_s'])['speed_mps'],0)
        self.assertAlmostEqual(profile_state(p,p['duration_s'])['distance_m'],200+40*math.pi)

    def test_teleport_or_corner_rejected(self):
        for second in [line((2,0),(3,0)),line((1,0),(1,1))]:
            with self.assertRaises(ValueError):stopping_profile([line((0,0),(1,0)),second],1,1,1)

    def test_nonfinite_speed_limits_rejected(self):
        with self.assertRaises(ValueError):stopping_profile([line((0,0),(1,0))],math.nan,1,1)


if __name__=='__main__':unittest.main()
