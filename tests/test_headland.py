import math
import unittest
from berthing_planner.headland import reachable_boxes,scan_variants,select_variant_chain


class HeadlandTests(unittest.TestCase):
    def test_reach_rectangle_matches_normalized_bound(self):
        s,shape=next(reachable_boxes((0,0,0),20,10))
        p=shape['coordinates'][0]
        self.assertAlmostEqual(p[0][0],20*math.sin(s/20))
        self.assertAlmostEqual(p[2][1],20*(1-math.cos(s/20)))

    def test_straight_and_maximum_turn_inside_outer_box(self):
        for s,shape in reachable_boxes((0,0,0),20,40):
            p=shape['coordinates'][0]
            for x,y in [(s,0),(20*math.sin(s/20),20*(1-math.cos(s/20)))]:
                self.assertGreaterEqual(x+1e-12,p[0][0]);self.assertLessEqual(x,p[1][0]+1e-12)
                self.assertLessEqual(abs(y),p[2][1]+1e-12)

    def test_length_limit_respected(self):
        self.assertTrue(all(s<2 for s,_ in reachable_boxes((0,0,0),20,2)))

    def test_original_and_reversed_preserved(self):
        v=scan_variants([[0,0],[200,0]],20)
        self.assertEqual(v[0]['points'],[[0,0],[200,0]])
        self.assertEqual(v[1]['points'],[[200,0],[0,0]])

    def test_only_proven_end_retreats(self):
        v=scan_variants([[0,0],[200,0]],20,blocked_exit=True)
        first=next(x for x in v if x['kind']=='retreated')
        self.assertEqual(first['points'][0],[0,0]);self.assertEqual(first['points'][1],[190,0])

    def test_short_scan_can_be_a_tangent_through_pass(self):
        v=scan_variants([[0,0],[2.5,0]],20,tangent_heading=math.pi/2)
        self.assertTrue(any(abs(x['heading_rad']-math.pi/2)<1e-9 and x['length_m']==40 for x in v))
        self.assertTrue(all(x['length_m']>0 for x in v))

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):scan_variants([[0,0],[0,0]],20)
        with self.assertRaises(ValueError):list(reachable_boxes((0,0,0),0,10))

    def test_layered_search_keeps_alternative_to_greedy_dead_end(self):
        layers=[[{'node_cost':0}],[{'node_cost':0},{'node_cost':1}],[{'node_cost':0}]]
        def edge(k,i,j,a,b):
            return None if k==2 and i==0 else {'length_m':1}
        out=select_variant_chain(layers,edge)
        self.assertTrue(out['passed']);self.assertEqual(out['variant_indices'],[0,1,0])

    def test_missing_layer_does_not_skip_task(self):
        out=select_variant_chain([[{'node_cost':0}],[],[{'node_cost':0}]],lambda *x:None)
        self.assertFalse(out['passed']);self.assertEqual(out['empty_layers'],[1])

    def test_fallback_only_when_needed(self):
        out=select_variant_chain([[{'node_cost':0}],[{'node_cost':0}]],lambda *x:None,lambda *x:{'length_m':2})
        self.assertTrue(out['passed']);self.assertEqual(out['objective_cost'],2)


if __name__=='__main__':unittest.main()
