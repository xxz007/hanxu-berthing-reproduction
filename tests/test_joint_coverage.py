import math
import unittest
from berthing_planner.joint_coverage import expanded_scan_variants,task_order_candidates,plan_variant_order


class JointCoverageTests(unittest.TestCase):
    def test_short_scan_has_shifted_rotated_original_length_candidates(self):
        variants=expanded_scan_variants([[0,0],[7.5,0]],20)
        self.assertTrue(any(abs(v['length_m']-7.5)<1e-9 and abs(v['heading_rad']-math.pi/2)<1e-9 and v['points'][0][0]>4 for v in variants))

    def test_retreat_can_be_reversed(self):
        variants=expanded_scan_variants([[0,0],[200,0]],20)
        self.assertTrue(any(v['points']==[[170,0],[10,0]]for v in variants))

    def test_candidate_geometry_lengths_and_deduplication(self):
        variants=expanded_scan_variants([[0,0],[52.5,0]],20)
        signatures=set()
        for v in variants:
            self.assertAlmostEqual(math.dist(*v['points']),v['length_m'])
            signature=tuple(round(x,8)for p in v['points']for x in p)
            self.assertNotIn(signature,signatures);signatures.add(signature)

    def test_orders_preserve_all_tasks(self):
        tasks=[{'task_id':str(i),'points':[[i,3-i],[i+1,3-i]]}for i in range(3)]
        orders=task_order_candidates(tasks)
        self.assertGreater(len(orders),1)
        for _,order in orders:self.assertCountEqual(order,['0','1','2'])

    def test_duplicate_task_identifier_rejected(self):
        t={'task_id':'x','points':[[0,0],[1,0]]}
        with self.assertRaises(ValueError):task_order_candidates([t,t])

    def test_expensive_search_waits_for_whole_fast_stage(self):
        calls=[];layers=[[{'node_cost':0},{'node_cost':1}],[{'node_cost':0}]]
        def fast(k,i,j,a,b):return {'length_m':1}if i==1 else None
        def slow(*args):calls.append(1);return {'length_m':2}
        out=plan_variant_order(layers,[fast,slow])
        self.assertTrue(out['passed']);self.assertEqual(out['variant_indices'],[1,0]);self.assertEqual(calls,[])

    def test_all_fast_failures_allow_next_stage(self):
        out=plan_variant_order([[{'node_cost':0}],[{'node_cost':0}]], [lambda *a:None,lambda *a:{'length_m':3}])
        self.assertTrue(out['passed']);self.assertEqual(out['objective_cost'],3)

    def test_short_cluster_can_be_visited_once_at_route_end(self):
        tasks=[{'task_id':'long','points':[[200,0],[400,0]]},{'task_id':'a','points':[[0,0],[20,0]]},
               {'task_id':'tiny','points':[[0,60],[2.5,60]]},{'task_id':'other','points':[[800,0],[1000,0]]}]
        orders=dict(task_order_candidates(tasks,20))
        self.assertEqual(orders['short_cluster_last_tiny'],['long','other','a','tiny'])
        self.assertEqual(orders['short_cluster_first_tiny'],['tiny','a','long','other'])


if __name__=='__main__':unittest.main()
