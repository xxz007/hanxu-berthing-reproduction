import copy
import math
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from audit_motion_feasibility import speed_checks, inflated_grid_hits, piece_points


class IndependentMotionAuditTests(unittest.TestCase):
    def setUp(self):
        self.pieces=[{'kind':'line','pose':[0,0,0],'length':100,'curvature':0}]
        self.cfg={'cruise_mps':2,'tangential_accel_mps2':1,'lateral_accel_mps2':.1}
        self.profile={'segments':[{'t0_s':0,'t1_s':52,'ramp_duration_s':2,'cruise_duration_s':48,
            'peak_speed_mps':2,'accel_mps2':1,'s0_m':0,'length_m':100}], 'duration_s':52,'length_m':100}

    def test_hand_solution_speed_law(self):
        self.assertEqual(speed_checks(self.profile,self.pieces,self.cfg),[])

    def test_changed_clock_is_rejected(self):
        self.profile['segments'][0]['t1_s']=51
        self.assertTrue(speed_checks(self.profile,self.pieces,self.cfg))

    def test_changed_speed_is_rejected(self):
        self.profile['segments'][0]['peak_speed_mps']=3
        self.assertTrue(speed_checks(self.profile,self.pieces,self.cfg))

    def test_lateral_acceleration_violation(self):
        self.pieces[0]['curvature']=1
        self.assertTrue(speed_checks(self.profile,self.pieces,self.cfg))

    def test_inflated_slab_catches_between_chord_and_arc(self):
        blocked=np.zeros((4,4),dtype=bool);blocked[1,1]=True
        self.assertEqual(inflated_grid_hits((.1,.95),(2.9,.95),blocked,[0,0,4,4],0),0)
        self.assertEqual(inflated_grid_hits((.1,.95),(2.9,.95),blocked,[0,0,4,4],.06),1)

    def test_touching_closed_grid_edge_is_collision(self):
        blocked=np.zeros((4,4),dtype=bool);blocked[1,1]=True
        self.assertEqual(inflated_grid_hits((.1,1),(2.9,1),blocked,[0,0,4,4],0),1)

    def test_independent_circle_rotation(self):
        p={'pose':[0,0,0],'length':20*math.pi,'curvature':1/20}
        expected=np.array([[0,0],[20,20],[0,40]])
        np.testing.assert_allclose(piece_points(p,[0,.5,1]),expected,atol=1e-12)

    def test_nan_phase_rejected(self):
        self.profile['segments'][0]['accel_mps2']=math.nan
        self.assertTrue(speed_checks(self.profile,self.pieces,self.cfg))


if __name__=='__main__':unittest.main()
