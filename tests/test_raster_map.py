import math
import unittest
from berthing_planner.raster_map import RasterQuadTree
from berthing_planner.search import astar


class RasterMapTests(unittest.TestCase):
    def test_known_single_obstacle_partition_and_axis(self):
        rows=[[0]*8 for _ in range(8)];rows[3][3]=1
        tree=RasterQuadTree({'schema_version':1,'status':'reconstructed_centerline_not_author_raw_map','bounds':[0,0,8,8],'n':8,'rows_bottom_to_top':rows})
        self.assertEqual(len(tree.leaves),10)
        self.assertTrue(tree.locate((3.5,3.5)).blocked)
        self.assertFalse(tree.locate((3.5,4.5)).blocked)
        self.assertFalse(tree.line_of_sight((1,1),(6,6)))

    def test_invalid_occupancy_rejected(self):
        with self.assertRaises(ValueError):
            RasterQuadTree({'schema_version':1,'status':'reconstructed_centerline_not_author_raw_map','bounds':[0,0,8,8],'n':3,'rows_bottom_to_top':[[0]*3]*3})

    def test_paper_parent_only_interpretation_has_no_corner_fallback(self):
        from berthing_planner.quadtree import QuadTree
        tree=QuadTree((0,0,8,8),[(3,0,4,6)],3)
        result=astar(tree,(1,1),(6,1),parent_policy='theta_relaxation',record_trace=True)
        self.assertGreater(result['length_m'],5)
        self.assertTrue(result['trace'])
        with self.assertRaisesRegex(ValueError,'No path'):
            astar(tree,(1,1),(6,1),parent_policy='paper_parent_only')

    def test_plain_astar_agrees_with_zero_heuristic(self):
        from berthing_planner.quadtree import QuadTree
        tree=QuadTree((0,0,8,8),[(3,0,4,6)],3)
        a=astar(tree,(1,1),(6,1))
        d=astar(tree,(1,1),(6,1),heuristic_weight=0)
        self.assertAlmostEqual(a['length_m'],d['length_m'])


if __name__=='__main__':
    unittest.main()
