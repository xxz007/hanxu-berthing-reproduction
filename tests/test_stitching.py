import math
import unittest
from berthing_planner.curves import Piece,line,g2_smooth,collision_free
from berthing_planner.stitching import piece_length,spine_anchor,stitch_rs


class StitchingTests(unittest.TestCase):
    def test_length_against_exact_nonuniform_straight_cubic_and_arc(self):
        p=Piece('cubic',controls=((0,0),(0,0),(1,0),(8,0)))
        self.assertAlmostEqual(piece_length(p),8)
        a=spine_anchor([p],[8],4)
        self.assertAlmostEqual(a['pose'][0],4,places=8)
        arc=Piece('arc',pose=(0,0,0),length=math.pi*200,curvature=.005)
        self.assertAlmostEqual(piece_length(arc),math.pi*200)

    def test_anchor_crosses_piece_boundary(self):
        pieces=[line((0,0),(100,0)),line((100,0),(100,100))]
        a=spine_anchor(pieces,[100,100],150)
        self.assertEqual(a['index'],1)
        self.assertEqual(a['pose'][:2],(100,50))
        with self.assertRaises(ValueError):
            spine_anchor(pieces,[100,100],201)

    def test_discarded_colliding_tail_does_not_reject_safe_combination(self):
        spine=[line((0,0),(1500,0)),line((1500,0),(1500,500))]
        obstacles=[(1490,250,1510,300)]
        self.assertFalse(collision_free(spine,obstacles,(-100,-100,2200,1000)))
        r=stitch_rs(spine,(0,0,0),(1800,0,0),.005,obstacles,(-100,-100,2200,1000),1800)
        self.assertEqual(r['status'],'retained_route_passed')
        self.assertTrue(collision_free(r['pieces'],obstacles,(-100,-100,2200,1000)))
        self.assertAlmostEqual(r['length_m'],1800)

    def test_colliding_retained_route_is_rejected(self):
        r=stitch_rs([line((0,0),(1800,0))],(0,0,0),(1800,0,0),.005,
                    [(500,-100,600,100)],(-100,-100,2200,100),1800)
        self.assertEqual(r['status'],'no_combination_in_this_finite_candidate_set')


if __name__=='__main__':
    unittest.main()
