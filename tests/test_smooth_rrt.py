import math
import unittest
from berthing_planner.quadtree import QuadTree
from berthing_planner.curves import wrap
from berthing_planner.smooth_rrt import steer,greedy_path,smooth_rrt


class SmoothRRTTests(unittest.TestCase):
    def test_steering_clips_signed_angle_across_pi(self):
        target=(100*math.cos(math.radians(-170)),100*math.sin(math.radians(-170)))
        point,yaw=steer((0,0),math.radians(170),target,5,math.radians(15))
        self.assertAlmostEqual(yaw,math.radians(-175))
        self.assertAlmostEqual(math.hypot(*point),5)

    def test_fixed_seed_open_map_path_and_turn_limit(self):
        tree=QuadTree((0,0,120,120),[],3)
        args=dict(step=5,maximum_turn=math.pi/3,max_iterations=1000,seed=42)
        a=smooth_rrt(tree,(5,5),(100,100),**args);b=smooth_rrt(tree,(5,5),(100,100),**args)
        self.assertEqual(a,b)
        self.assertEqual(a['status'],'path_found')
        self.assertLessEqual(a['maximum_actual_turn_rad'],math.pi/3+1e-10)
        self.assertEqual(a['path'][0],(5,5));self.assertEqual(a['path'][-1],(100,100))
        self.assertTrue(all(tree.line_of_sight(p,q) for p,q in zip(a['path'],a['path'][1:])))

    def test_greedy_future_join_guard_prevents_stranding(self):
        tree=QuadTree((-1,-1,40,40),[],2)
        p=[(0,0),(10,0),(20,10/math.sqrt(3)),(20+10/math.sqrt(3),10/math.sqrt(3)+10),
           (20+10/math.sqrt(3),10/math.sqrt(3)+20)]
        literal=greedy_path(tree,p,0,math.pi/6,'incoming_only')
        self.assertEqual(literal['status'],'stalled_after_shortcut')
        safe=greedy_path(tree,p,0,math.pi/6,'preserve_both_joins')
        self.assertEqual(safe['status'],'smoothed_polyline')
        yaw=0
        for a,b in zip(safe['path'],safe['path'][1:]):
            new=math.atan2(b[1]-a[1],b[0]-a[0]);self.assertLessEqual(abs(wrap(new-yaw)),math.pi/6+1e-10);yaw=new

    def test_wall_retains_budget_failure(self):
        tree=QuadTree((0,0,120,120),[(50,0,60,120)],5)
        r=smooth_rrt(tree,(5,5),(100,100),step=5,max_iterations=100,seed=2)
        self.assertEqual(r['status'],'iteration_budget_exhausted');self.assertFalse(r['path'])


if __name__=='__main__':
    unittest.main()
