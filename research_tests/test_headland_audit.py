import math
import unittest
import numpy as np
from chart_geometry import Geos
from berthing_planner.curves import Piece
from berthing_planner.dubins import candidates
from berthing_planner.headland import reachable_boxes
from run_headland_connectors import certify
from audit_headland_navigation import arc_enclosure
from audit_motion_feasibility import piece_points


class HeadlandIndependentTests(unittest.TestCase):
    def test_mixed_bang_bang_controls_inside_reachable_bound(self):
        rng=np.random.default_rng(573)
        # Independent midpoint integration, arbitrary sign-changing curvature.
        for s,shape in reachable_boxes((0,0,0),20,40):
            control=rng.uniform(-1/20,1/20,(100,2000));ds=s/2000
            yaw=np.cumsum(control*ds,axis=1)-control*ds/2
            x=np.sum(np.cos(yaw)*ds,axis=1);y=np.sum(np.sin(yaw)*ds,axis=1)
            box=shape['coordinates'][0]
            self.assertTrue(np.all(x>=box[0][0]-1e-6));self.assertTrue(np.all(x<=s+1e-6))
            self.assertTrue(np.all(np.abs(y)<=box[2][1]+1e-6))

    def test_independent_circle_endpoint_for_both_turns(self):
        for sign in (-1,1):
            p=candidates((0,0,0),(20,sign*20,sign*math.pi/2),20)[0]
            endpoint=piece_points(p['pieces'][0].__dict__,[1])[0]
            np.testing.assert_allclose(endpoint,[20,sign*20],atol=1e-8)
            self.assertAlmostEqual(p['length_m'],10*math.pi)

    def test_enclosure_bound_contains_analytic_arc(self):
        p={'pose':[0,0,.3],'length':180,'curvature':-.05,'direction':1,'kind':'arc'}
        xy,bound=arc_enclosure([p]);xy=np.asarray(xy)
        fine=piece_points(p,np.linspace(0,1,1001));distance=np.full(len(fine),np.inf)
        for a,b in zip(xy,xy[1:]):
            d=b-a;t=np.clip((fine-a)@d/(d@d),0,1)
            distance=np.minimum(distance,np.linalg.norm(fine-a-t[:,None]*d,axis=1))
        self.assertLessEqual(np.max(distance),bound+1e-10)

    def test_endpoints_inside_is_insufficient_for_curve_safety(self):
        # Chord diagonal fits in a thin strip, but a quarter circle bows out.
        domain={'type':'Polygon','coordinates':[[[-2,-2],[22,18],[22,22],[18,22],[-2,2],[-2,-2]]]}
        hazards={'type':'Point','coordinates':[1000,1000]}
        curve=[Piece('arc',pose=(0,0,0),length=10*math.pi,curvature=.05)]
        with Geos()as g:
            self.assertTrue(g.covers(domain,{'type':'LineString','coordinates':[[0,0],[20,20]]}))
            self.assertFalse(certify(g,domain,hazards,curve,.02)['passed'])

    def test_clear_straight_connector_passes(self):
        domain={'type':'Polygon','coordinates':[[[-10,-10],[110,-10],[110,10],[-10,10],[-10,-10]]]}
        with Geos()as g:
            result=certify(g,domain,{'type':'Point','coordinates':[1000,1000]},[Piece('line',pose=(0,0,0),length=100,curvature=0)],.02)
            self.assertTrue(result['passed']);self.assertGreater(result['minimum_hazard_clearance_lower_bound_m'],50)


if __name__=='__main__':unittest.main()
