import copy
import math
import unittest
import numpy as np
from berthing_planner.curves import Piece
from berthing_planner.motion_feasibility import stopping_profile
from berthing_planner.curved_timing import CurvedClock,continuous_distance_bounds
from audit_curved_cooperation import IndependentClock,audit_partition


class IndependentCurvedClockTests(unittest.TestCase):
    def fixture(self):
        p=Piece('arc',pose=(0,0,0),length=10*math.pi,curvature=.05);profile=stopping_profile([p],2,.5,.5)
        body={'pieces':[p.__dict__],'profile':profile,'departure_s':7}
        hold={'pieces':[],'profile':{'segments':[],'duration_s':0.,'length_m':0.},'initial_pose':[80,70,0]}
        return body,hold

    def test_independent_phase_integrals_and_circle_positions(self):
        body,_=self.fixture();native=CurvedClock(body['pieces'],body['profile'],7);audit=IndependentClock(body)
        for t in np.linspace(0,native.finish_s+5,101):
            p,s,v,yaw=audit.state(t);state=native.state(t)
            np.testing.assert_allclose(p,state['position_m'],atol=1e-10);self.assertAlmostEqual(s,state['distance_m']);self.assertAlmostEqual(v,state['speed_mps'])

    def test_whole_interval_certificate_independently_passes(self):
        body,hold=self.fixture();native=CurvedClock(body['pieces'],body['profile'],7);fixed=CurvedClock.stationary(hold['initial_pose'])
        report=continuous_distance_bounds(native,fixed,native.finish_s+5)
        self.assertTrue(audit_partition(report,IndependentClock(body),IndependentClock(hold),native.finish_s+5)['passed'])

    def test_missing_interval_rejected(self):
        body,hold=self.fixture();native=CurvedClock(body['pieces'],body['profile'],7);report=continuous_distance_bounds(native,CurvedClock.stationary(hold['initial_pose']),native.finish_s+5)
        report['partition'].pop(len(report['partition'])//2)
        self.assertFalse(audit_partition(report,IndependentClock(body),IndependentClock(hold),native.finish_s+5)['passed'])

    def test_false_zero_movement_bound_rejected(self):
        body,hold=self.fixture();native=CurvedClock(body['pieces'],body['profile'],7);report=continuous_distance_bounds(native,CurvedClock.stationary(hold['initial_pose']),native.finish_s+5)
        row=max(report['partition'],key=lambda r:r['relative_movement_bound_m']);row['relative_movement_bound_m']=0.
        self.assertFalse(audit_partition(report,IndependentClock(body),IndependentClock(hold),native.finish_s+5)['passed'])

    def test_false_witness_rejected(self):
        body,hold=self.fixture();native=CurvedClock(body['pieces'],body['profile'],7);report=continuous_distance_bounds(native,CurvedClock.stationary(hold['initial_pose']),native.finish_s+5);report['minimum_distance_witness_m']-=1
        self.assertFalse(audit_partition(report,IndependentClock(body),IndependentClock(hold),native.finish_s+5)['passed'])


if __name__=='__main__':unittest.main()
