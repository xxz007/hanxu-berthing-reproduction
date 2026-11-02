import math
import unittest
import numpy as np
from audit_joint_coverage import distance_to_piece,straight_capsule_upper
from chart_geometry import Geos


class ExactSensorDistanceTests(unittest.TestCase):
    def test_line_distance_interior_and_caps(self):
        p={'pose':[0,0,0],'length':10,'curvature':0}
        np.testing.assert_allclose(distance_to_piece(np.array([[5,3],[-4,0],[13,4]]),p),[3,4,5])

    def test_quarter_circle_sector_and_endpoint(self):
        p={'pose':[0,0,0],'length':10*math.pi,'curvature':.05}
        xy=np.array([[20,20],[0,20],[0,40],[10*math.sqrt(2),20-10*math.sqrt(2)]])
        np.testing.assert_allclose(distance_to_piece(xy,p),[0,20,20*math.sqrt(2),0],atol=1e-8)

    def test_right_turn_and_full_circle(self):
        p={'pose':[0,0,0],'length':40*math.pi,'curvature':-.05}
        np.testing.assert_allclose(distance_to_piece(np.array([[0,-20],[20,-20],[-20,-20]]),p),[20,0,0],atol=1e-8)

    def test_independent_capsule_upper_has_closed_valid_rings(self):
        with Geos()as g:
            shape=straight_capsule_upper(g,[[0,0],[100,0]],50)
            self.assertTrue(g.measure(shape)['valid']);self.assertGreater(g.measure(shape)['area_m2'],10000+math.pi*2500)


if __name__=='__main__':unittest.main()
