import math
import unittest
from berthing_planner.curves import g2_smooth,curvature_maximum,wrap,collision_free
from berthing_planner.g2_bisection import bisect_corners


class BisectionTests(unittest.TestCase):
    def test_known_right_angle_that_basic_pair_cannot_fit(self):
        path=[(-300,0),(0,0),(0,300)]
        with self.assertRaisesRegex(ValueError,'overlap|exceeds'):
            g2_smooth(path,.005)
        r=bisect_corners(path,.005)
        self.assertEqual(r['status'],'constructed')
        self.assertEqual(sum(p.kind=='cubic' for p in r['pieces']),4)
        corner=r['corners'][0]
        # Fig.8 chord must accommodate exactly one trim from each sub-corner.
        self.assertAlmostEqual(math.dist(*corner['virtual_vertices']),2*corner['d_b_m'])
        self.assertLess(corner['required_trim_m'],300)
        printed_d=(1+math.sqrt(2))*1.1228*math.sin(math.pi/8)/(.005*math.cos(math.pi/8)**2)
        self.assertAlmostEqual(corner['required_trim_m'],printed_d,delta=.1)
        self.assertLess(math.dist(r['pieces'][0].evaluate(0)[:2],path[0]),1e-10)
        self.assertLess(math.dist(r['pieces'][-1].evaluate(1)[:2],path[-1]),1e-10)

    def test_angles_reflections_curvature_and_all_joins(self):
        for angle in (.05,.4,math.pi/2,2.5,2.9):
            for sign in (-1,1):
                path=[(-100000,0),(0,0),(100000*math.cos(angle),sign*100000*math.sin(angle))]
                r=bisect_corners(path,.005)
                self.assertEqual(r['status'],'constructed')
                for piece in r['pieces']:
                    self.assertLessEqual(curvature_maximum(piece),.005+1e-11)
                for a,b in zip(r['pieces'],r['pieces'][1:]):
                    p,q=a.evaluate(1),b.evaluate(0)
                    self.assertLess(math.dist(p[:2],q[:2]),1e-8)
                    self.assertLess(abs(wrap(p[2]-q[2])),1e-9)
                    self.assertLess(abs(p[3]-q[3]),1e-10)

    def test_unresolved_short_segment_is_explicit_failure(self):
        r=bisect_corners([(-100,0),(0,0),(0,100)],.005)
        self.assertEqual(r['status'],'insufficient_segment_budget')
        self.assertEqual(r['insufficient_segments'],[0,1])
        self.assertFalse(r['pieces'])

    def test_shared_budget_not_only_each_corner(self):
        r=bisect_corners([(-1000,0),(0,0),(0,400),(1000,400)],.005)
        self.assertEqual(r['insufficient_segments'],[1])

    def test_invalid_inputs_rejected(self):
        for path,k in [([], .005),([(0,0)],.005),([(0,0),(0,0)],.005),
                       ([(0,0),(1,0)],0),([(0,0),(float('nan'),1)],.005),
                       ([(0,0),(1,0),(0,0)],.005)]:
            with self.assertRaises(ValueError):
                bisect_corners(path,k)

    def test_straight_input_and_collision_are_separate_checks(self):
        r=bisect_corners([(0,0),(1,0),(2,0)],.005)
        self.assertEqual(r['status'],'constructed')
        self.assertTrue(all(curvature_maximum(p)==0 for p in r['pieces']))
        self.assertFalse(collision_free(r['pieces'],[(.9,-.1,1.1,.1)],(-1,-1,3,1)))


if __name__=='__main__':
    unittest.main()
