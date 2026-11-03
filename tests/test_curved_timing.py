import math
import unittest
from berthing_planner.curves import Piece
from berthing_planner.motion_feasibility import stopping_profile,profile_state
from berthing_planner.curved_timing import CurvedClock,continuous_distance_bounds


def clock(piece,delay=0):return CurvedClock([piece],stopping_profile([piece],2,.5,.5),delay)


class CurvedTimingTests(unittest.TestCase):
    def test_clock_matches_known_profile_and_holds(self):
        p=Piece('line',pose=(0,0,0),length=100,curvature=0);c=clock(p,10)
        for t in(0,10,11,14,30,61,100):
            s=c.state(t);reference=profile_state(c.profile,t-10)
            self.assertAlmostEqual(s['position_m'][0],reference['distance_m']);self.assertAlmostEqual(s['speed_mps'],reference['speed_mps'])

    def test_quarter_circle_midpoint(self):
        p=Piece('arc',pose=(0,0,0),length=10*math.pi,curvature=.05);c=clock(p)
        pose=c.state(c.duration_s/2)['position_m'];self.assertAlmostEqual(pose[0],10*math.sqrt(2));self.assertAlmostEqual(pose[1],20-10*math.sqrt(2))

    def test_stationary_pair_exact_distance_is_enclosed(self):
        out=continuous_distance_bounds(CurvedClock.stationary((0,0,0)),CurvedClock.stationary((3,4,0)),100)
        self.assertTrue(out['converged']);self.assertLessEqual(out['minimum_distance_lower_m'],5);self.assertGreaterEqual(out['maximum_distance_upper_m'],5)

    def test_crossing_between_sparse_frames_is_detected(self):
        a=clock(Piece('line',pose=(-10,0,0),length=20));b=clock(Piece('line',pose=(0,-10,math.pi/2),length=20))
        out=continuous_distance_bounds(a,b,a.finish_s)
        self.assertTrue(out['converged']);self.assertLess(out['minimum_distance_witness_m'],1e-8);self.assertGreater(out['maximum_distance_witness_m'],14)

    def test_constant_parallel_gap_certified(self):
        a=clock(Piece('line',pose=(0,0,0),length=100));b=clock(Piece('line',pose=(0,40,0),length=100))
        out=continuous_distance_bounds(a,b,a.finish_s,tolerance_m=.1)
        self.assertTrue(out['converged']);self.assertGreater(out['minimum_distance_lower_m'],39.89);self.assertLess(out['maximum_distance_upper_m'],40.11)

    def test_budget_exhaustion_cannot_claim_convergence(self):
        a=clock(Piece('line',pose=(0,0,0),length=100));b=CurvedClock.stationary((50,40,0))
        self.assertFalse(continuous_distance_bounds(a,b,a.finish_s,max_refinements=0)['converged'])

    def test_profile_jump_and_invalid_integrals_rejected(self):
        p=Piece('line',pose=(0,0,0),length=100);profile=stopping_profile([p],2,.5,.5);profile['segments'][0]['s0_m']=1
        with self.assertRaises(ValueError):CurvedClock([p],profile)

    def test_geometric_position_jump_rejected(self):
        first=Piece('line',pose=(0,0,0),length=10);second=Piece('line',pose=(10,0,0),length=10)
        profile=stopping_profile([first,second],2,.5,.5);second.pose=(11,0,0)
        with self.assertRaises(ValueError):CurvedClock([first,second],profile)

    def test_arc_collision_away_from_temporal_midpoint(self):
        moving=clock(Piece('arc',pose=(0,0,0),length=10*math.pi,curvature=.05))
        fixed=CurvedClock.stationary((10,20-10*math.sqrt(3),0))
        out=continuous_distance_bounds(moving,fixed,moving.finish_s)
        self.assertTrue(out['converged']);self.assertLess(out['minimum_distance_witness_m'],.05);self.assertEqual(out['minimum_distance_lower_m'],0)

    def test_holding_vehicle_can_be_hit_between_saved_frames(self):
        moving=clock(Piece('line',pose=(-13,0,0),length=50));fixed=CurvedClock.stationary((0,0,0))
        out=continuous_distance_bounds(moving,fixed,moving.finish_s)
        self.assertTrue(out['converged']);self.assertLess(out['minimum_distance_witness_m'],.05)

    def test_exact_threshold_keeps_outward_numerical_margin(self):
        out=continuous_distance_bounds(CurvedClock.stationary((0,0,0)),CurvedClock.stationary((25,0,0)),100)
        self.assertEqual(out['minimum_distance_witness_m'],25);self.assertLess(out['minimum_distance_lower_m'],25)


if __name__=='__main__':unittest.main()
