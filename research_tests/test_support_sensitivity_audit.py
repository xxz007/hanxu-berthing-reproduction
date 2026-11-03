import math
import unittest
import numpy as np
from berthing_planner.curves import Piece
from berthing_planner.motion_feasibility import stopping_profile
from berthing_planner.curved_timing import CurvedClock,continuous_distance_bounds
from berthing_planner.scheduled_timing import ScheduledCurvedClock,WindowClock,classify_support_range
from audit_support_sensitivity import IndependentSchedule,IndependentWindow,classify_independently
from audit_curved_cooperation import IndependentClock,audit_partition


class ScheduledAuditTests(unittest.TestCase):
    def fixture(self):
        first=Piece('arc',pose=(0,0,0),length=10*math.pi,curvature=.05)
        second=Piece('line',pose=(20,20,math.pi/2),length=30)
        return [{'pieces':[p.__dict__],'profile':stopping_profile([p],2,.5,.5),'departure_s':d}for p,d in [(first,5),(second,80)]]

    def test_401_states_independent_arcs_holds_and_second_leg(self):
        legs=self.fixture();c=ScheduledCurvedClock(legs,(0,0,0));other=IndependentSchedule(legs,(0,0,0))
        for t in np.linspace(0,140,401):
            p,s,v,y=other.state(t);a=c.state(t);np.testing.assert_allclose(p,a['position_m'],atol=1e-10);self.assertAlmostEqual(s,a['distance_m']);self.assertAlmostEqual(v,a['speed_mps']);self.assertAlmostEqual(math.remainder(y-a['heading_rad'],2*math.pi),0)

    def test_window_certificate_with_wait_and_second_departure(self):
        legs=self.fixture();c=ScheduledCurvedClock(legs,(0,0,0));fixed=CurvedClock.stationary((100,90,0));out=continuous_distance_bounds(WindowClock(c,60,125),fixed,65)
        audit=audit_partition(out,IndependentWindow(IndependentSchedule(legs,(0,0,0)),60),IndependentClock({'pieces':[],'profile':{'segments':[],'duration_s':0,'length_m':0},'initial_pose':[100,90,0]}),65)
        self.assertTrue(audit['passed'])

    def test_independent_schedule_cannot_omit_interleg_distance(self):
        c=IndependentSchedule(self.fixture(),(0,0,0));self.assertAlmostEqual(c.state(140)[1],10*math.pi+30);self.assertAlmostEqual(c.state(70)[1],10*math.pi)

    def test_empty_schedule_is_initial_hold(self):
        c=IndependentSchedule([],(3,4,0));np.testing.assert_array_equal(c.state(100)[0],[3,4]);self.assertEqual(c.state(100)[1:3],(0,0))

    def test_three_outcomes_and_uncertainty_independently_match(self):
        p={'converged':True,'minimum_distance_lower_m':40,'minimum_distance_witness_m':40.04,'maximum_distance_witness_m':2295.42,'maximum_distance_upper_m':2295.48}
        expected={2280:'initial_condition_impossible',2290:'witnessed_strategy_violation',2295.45:'unresolved_bounds',2300:'certified_pass'}
        for radius,status in expected.items():
            self.assertEqual(classify_independently([p],radius,25,2289.56),status);self.assertEqual(classify_support_range([p],radius,25,2289.56),status)

    def test_range_pass_cannot_hide_separation_violation(self):
        p={'converged':True,'minimum_distance_lower_m':4.95,'minimum_distance_witness_m':5,'maximum_distance_witness_m':2200,'maximum_distance_upper_m':2200.05}
        self.assertEqual(classify_independently([p],3000,25,2289.56),'witnessed_strategy_violation')


if __name__=='__main__':unittest.main()
