import json
import math
from pathlib import Path
import random
import unittest

from berthing_planner.quadtree import QuadTree
from berthing_planner.search import astar
from berthing_planner.pipeline import UniformGrid,trim,run,validate_config
from berthing_planner.curves import Piece,line,g2_smooth,curvature_maximum,collision_free,polynomial_roots_unit,wrap
from berthing_planner.reeds_shepp import candidates,connect

ROOT = Path(__file__).resolve().parents[1]


class PlanningTests(unittest.TestCase):
    def test_uniform_known_shortest_and_any_angle(self):
        grid = UniformGrid((0,0,4,4),[],2)
        ordinary = astar(grid,(.5,.5),(3.5,3.5))
        improved = astar(grid,(.5,.5),(3.5,3.5),True)
        self.assertAlmostEqual(ordinary['length_m'],6)
        self.assertAlmostEqual(improved['length_m'],math.sqrt(18))

    def test_no_path_is_reported(self):
        tree = QuadTree((0,0,8,8),[(3,0,4,8)],3)
        with self.assertRaisesRegex(ValueError,'No path'):
            astar(tree,(1,1),(6,6),True)

    def test_start_inside_obstacle_rejected(self):
        with self.assertRaisesRegex(ValueError,'blocked'):
            astar(QuadTree((0,0,4,4),[(0,0,1,1)],2),(.5,.5),(3,3))

    def test_start_equals_goal(self):
        self.assertEqual(astar(QuadTree((0,0,4,4),[],2),(1,1),(1,1))['length_m'],0)

    def test_g2_right_angle_curvature_and_joins(self):
        pieces = g2_smooth([(0,0),(1000,0),(1000,1000)],.005)
        maximum = max(curvature_maximum(p) for p in pieces)
        self.assertLessEqual(maximum,.005)
        self.assertGreater(maximum,.0049)
        for a,b in zip(pieces,pieces[1:]):
            p,q = a.evaluate(1),b.evaluate(0)
            self.assertLess(math.dist(p[:2],q[:2]),1e-9)
            self.assertLess(abs(wrap(p[2]-q[2])),1e-9)
            self.assertLess(abs(p[3]-q[3]),1e-9)

    def test_g2_short_corners_reject_instead_of_violating_radius(self):
        with self.assertRaisesRegex(ValueError,'overlap|exceeds'):
            g2_smooth([(0,0),(10,0),(10,10)],.005)

    def test_g2_collinear_is_straight(self):
        self.assertEqual(max(curvature_maximum(p) for p in g2_smooth([(0,0),(1,0),(2,0)],.005)),0)

    def test_polynomial_even_and_simple_roots(self):
        roots = polynomial_roots_unit([-.046875,.4375,-1.25,1])
        self.assertTrue(any(abs(r-.25)<1e-6 for r in roots))
        self.assertTrue(any(abs(r-.75)<1e-6 for r in roots))

    def test_cubic_collision_between_endpoints(self):
        p = Piece('cubic',controls=((1,1),(1,7),(7,7),(7,1)))
        self.assertFalse(collision_free([p],[(3,5,5,6)],(0,0,8,8)))
        self.assertTrue(collision_free([p],[(3,0,5,.5)],(0,0,8,8)))

    def test_arc_collision_near_midpoint(self):
        p = Piece('arc',pose=(2,2,0),length=math.pi,curvature=1)
        self.assertFalse(collision_free([p],[(2.9,2.9,3.1,3.1)],(0,0,8,8)))

    def test_obstacle_contact_is_collision(self):
        self.assertFalse(collision_free([line((1,1),(7,1))],[(3,1,4,2)],(0,0,8,8)))

    def test_rs_known_forward_and_reverse(self):
        for target,sign in (((1000,0,0),1),((-1000,0,0),-1)):
            paths = candidates((0,0,0),target,.005,final_direction=sign)
            self.assertTrue(paths)
            self.assertAlmostEqual(paths[0]['length_m'],1000)
            self.assertEqual(paths[0]['pieces'][-1].direction,sign)

    def test_rs_random_pose_endpoints_and_radius(self):
        rng = random.Random(20261029)
        for _ in range(40):
            start = (rng.uniform(-500,500),rng.uniform(-500,500),rng.uniform(-math.pi,math.pi))
            goal = (rng.uniform(-500,500),rng.uniform(-500,500),rng.uniform(-math.pi,math.pi))
            paths = candidates(start,goal,.005)
            self.assertTrue(paths)
            for p in paths:
                end = p['pieces'][-1].evaluate(1)
                self.assertLess(math.dist(end[:2],goal[:2]),1e-5)
                self.assertLess(abs(wrap(end[2]-goal[2])),1e-7)
                self.assertLessEqual(max(abs(q.curvature) for q in p['pieces']),.005)

    def test_rs_direction_filter_before_candidate_selection(self):
        p = connect((2900,2800,.08),(3330,3500,0),.005,[],(0,0,4800,4800),2,-1,1)
        self.assertIsNotNone(p)
        self.assertEqual((p['pieces'][0].direction,p['pieces'][-1].direction),(1,-1))

    def test_exact_cubic_trim(self):
        p = Piece('cubic',controls=((0,0),(1,0),(2,1),(3,1)))
        q = trim([p],0,.2,0,.8)[0]
        for t in (0,.1,.5,1):
            a,b = q.evaluate(t),p.evaluate(.2+.6*t)
            self.assertLess(math.dist(a[:2],b[:2]),1e-12)
            self.assertLess(abs(a[3]-b[3]),1e-12)

    def test_invalid_units_rejected_at_boundary(self):
        config=json.loads((ROOT/'configs/harbor_demo.json').read_text())
        config['units']['angle']='degrees'
        with self.assertRaises(ValueError):
            validate_config(config)

    def test_full_synthetic_pipeline(self):
        config=json.loads((ROOT/'configs/harbor_demo.json').read_text())
        tree,grid,_,results=run(config)
        self.assertLess(len(tree.leaves),len(grid.leaves))
        for result in results:
            m=result['metrics']
            self.assertTrue(m['validation_passed'])
            self.assertLessEqual(m['maximum_curvature_extrema'],.0050005)
            self.assertGreaterEqual(m['end_anchor_m'],2*m['spine_length_m']/3)
            self.assertLess(result['improved']['length_m'],result['multi']['length_m'])
        self.assertGreater(results[1]['metrics']['reverse_m'],0)
        self.assertEqual(results[1]['metrics']['final_direction'],-1)
        self.assertGreaterEqual(results[1]['metrics']['cusps'],1)


if __name__=='__main__':
    unittest.main()
