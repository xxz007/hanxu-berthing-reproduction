import math
import unittest
from berthing_planner.curves import Piece
from berthing_planner.motion_feasibility import stopping_profile
from berthing_planner.curved_timing import CurvedClock,continuous_distance_bounds
from berthing_planner.scheduled_timing import ScheduledCurvedClock,WindowClock,classify_support_range


def leg(x,length,departure):
    p=Piece('line',pose=(x,0,0),length=length)
    return {'pieces':[p.__dict__],'profile':stopping_profile([p],2,.5,.5),'departure_s':departure}


class ScheduledTimingTests(unittest.TestCase):
    def test_explicit_interleg_and_final_holds(self):
        c=ScheduledCurvedClock([leg(0,10,5),leg(10,10,40)],(0,0,0))
        self.assertEqual(c.state(0)['mode'],'predeparture_hold');self.assertEqual(c.state(30)['mode'],'interleg_hold')
        self.assertEqual(c.state(30)['position_m'],(10.,0.));self.assertEqual(c.state(100)['mode'],'completed_hold')
        self.assertEqual(c.state(100)['distance_m'],20)

    def test_progress_includes_previous_legs(self):
        c=ScheduledCurvedClock([leg(0,10,0),leg(10,20,40)],(0,0,0))
        for t in(0,5,15,30,40,43,48,60,100):self.assertAlmostEqual(c.state(t)['position_m'][0],c.state(t)['distance_m'])

    def test_empty_schedule_stays_at_initial_pose(self):
        c=ScheduledCurvedClock([],(1,2,3));self.assertEqual(c.state(100)['position_m'],(1,2));self.assertEqual(c.length_m,0)

    def test_overlap_and_teleport_rejected(self):
        for legs in ([leg(0,10,0),leg(10,10,1)],[leg(0,10,0),leg(11,10,40)]):
            with self.assertRaises(ValueError):ScheduledCurvedClock(legs,(0,0,0))

    def test_heading_jump_rejected(self):
        b=leg(10,10,40);b['pieces'][0]['pose']=(10,0,math.pi/2)
        with self.assertRaises(ValueError):ScheduledCurvedClock([leg(0,10,0),b],(0,0,0))

    def test_hold_collision_between_sorties_detected(self):
        c=ScheduledCurvedClock([leg(0,10,0),leg(10,10,40)],(0,0,0));p=Piece('line',pose=(10,-10,math.pi/2),length=20)
        b=CurvedClock([p],stopping_profile([p],2,.5,.5),20)
        r=continuous_distance_bounds(c,b,65);self.assertLess(r['minimum_distance_witness_m'],1e-7)

    def test_window_uses_global_clock_and_monotone_progress(self):
        c=ScheduledCurvedClock([leg(0,10,0),leg(10,20,40)],(0,0,0));w=WindowClock(c,30,70)
        self.assertEqual(w.state(0)['distance_m'],0);self.assertAlmostEqual(w.state(20)['distance_m'],c.state(50)['distance_m']-10)
        self.assertIn(10,w.phase_times)

    def test_window_certificate_encloses_known_constant_gap(self):
        a=ScheduledCurvedClock([leg(0,10,0),leg(10,10,40)],(0,0,0));r=continuous_distance_bounds(WindowClock(a,25,35),CurvedClock.stationary((10,30,0)),10)
        self.assertLessEqual(r['minimum_distance_lower_m'],30);self.assertGreaterEqual(r['maximum_distance_upper_m'],30)

    def test_initial_condition_failure_has_distinct_classification(self):
        self.assertEqual(classify_support_range([],2200,25,2289),'initial_condition_impossible')

    def test_missing_certificates_never_pass(self):
        self.assertEqual(classify_support_range([],3000,25,2289),'unresolved_bounds')

    def test_bounds_straddling_threshold_is_unresolved(self):
        p={'converged':True,'minimum_distance_lower_m':30,'minimum_distance_witness_m':30.01,'maximum_distance_witness_m':2299.99,'maximum_distance_upper_m':2300.02}
        self.assertEqual(classify_support_range([p],2300,25,2289),'unresolved_bounds')
        self.assertEqual(classify_support_range([p],2301,25,2289),'certified_pass')
        self.assertEqual(classify_support_range([p],2299,25,2289),'witnessed_strategy_violation')


if __name__=='__main__':unittest.main()
