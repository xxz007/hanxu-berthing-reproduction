"""Independent checks against Yang/Sukkarieh printed equations (4)-(15)."""
import math
import unittest
from berthing_planner.curves import g2_smooth, curvature_maximum, wrap


class G2CBSSourceTests(unittest.TestCase):
    def test_printed_equation_trim_controls_and_join_gap(self):
        c2 = 2*(math.sqrt(6)-1)/5
        c3 = (c2+4)/(7.2364+6)
        for angle in (.1,.5,math.pi/2,2.5):
            beta = angle/2
            d = (c2+4)**2*math.sin(beta)/(54*c3*.005*math.cos(beta)**2)
            b = (0,0)
            end = (100000*math.cos(angle),100000*math.sin(angle))
            pieces = g2_smooth([(-100000,0),b,end],.005)
            first,second = [p for p in pieces if p.kind=='cubic']
            self.assertAlmostEqual(first.controls[0][0],-d)
            self.assertAlmostEqual(math.dist(second.controls[-1],b),d)
            self.assertAlmostEqual(math.dist(first.controls[0],first.controls[1]),c2*c3*d)
            self.assertAlmostEqual(math.dist(first.controls[1],first.controls[2]),c3*d)
            ud = (math.cos(beta),math.sin(beta))
            kb = 6*c3*d*math.cos(beta)/(c2+4)
            literal_b3 = tuple(first.controls[2][j]+kb*ud[j] for j in range(2))
            literal_e3 = tuple(second.controls[1][j]-kb*ud[j] for j in range(2))
            for j in range(2):
                self.assertAlmostEqual(first.controls[-1][j],(literal_b3[j]+literal_e3[j])/2)
            # Literal B3 and E3 do not exactly coincide with the printed c1.
            self.assertGreater(math.dist(literal_b3,literal_e3),0)
            self.assertLess(math.dist(literal_b3,literal_e3)/d,.0002)

    def test_curvature_bound_and_g2_across_turn_angles_and_reflections(self):
        for angle in (.01,.2,.7,math.pi/2,2.5,2.9):
            for sign in (-1,1):
                path = [(-100000,0),(0,0),(100000*math.cos(angle),sign*100000*math.sin(angle))]
                pieces = g2_smooth(path,.005)
                for piece in pieces:
                    self.assertLessEqual(curvature_maximum(piece),.005+1e-12)
                for a,b in zip(pieces,pieces[1:]):
                    p,q=a.evaluate(1),b.evaluate(0)
                    self.assertLess(math.dist(p[:2],q[:2]),1e-8)
                    self.assertLess(abs(wrap(p[2]-q[2])),1e-9)
                    self.assertLess(abs(p[3]-q[3]),1e-10)

    def test_shared_segment_requires_sum_of_both_trims(self):
        # Each 90-degree corner fits individually in the 300 m middle segment,
        # but their summed trims do not. No individual-corner-only acceptance.
        with self.assertRaisesRegex(ValueError,'overlap'):
            g2_smooth([(0,0),(1000,0),(1000,300),(2000,300)],.005)

    def test_radius_scaling(self):
        path=[(-10000,0),(0,0),(0,10000)]
        a=[p for p in g2_smooth(path,.005) if p.kind=='cubic'][0]
        b=[p for p in g2_smooth(path,.0025) if p.kind=='cubic'][0]
        self.assertAlmostEqual(math.dist(b.controls[0],(0,0)),2*math.dist(a.controls[0],(0,0)))
        self.assertAlmostEqual(curvature_maximum(b),curvature_maximum(a)/2)


if __name__=='__main__':
    unittest.main()
