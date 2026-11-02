import math
import unittest
from berthing_planner.curves import collision_free,wrap
from berthing_planner.maneuver_search import forward_pose_search


class ManeuverSearchTests(unittest.TestCase):
    def check_path(self,path,start,goal,radius,clear):
        self.assertTrue(path['passed']);pose=start
        for p in path['pieces']:
            self.assertLess(math.dist(pose[:2],p.pose[:2]),1e-7)
            self.assertLess(abs(wrap(pose[2]-p.pose[2])),1e-7)
            self.assertLessEqual(abs(p.curvature),1/radius+1e-12)
            self.assertEqual(p.direction,1);self.assertTrue(clear([p]));pose=p.evaluate(1)[:3]
        self.assertLess(math.dist(pose[:2],goal[:2]),1e-5);self.assertLess(abs(wrap(pose[2]-goal[2])),1e-7)

    def test_open_straight(self):
        out=forward_pose_search((0,0,0),(50,0,0),5,lambda p:True,bounds=(-100,-100,100,100))
        self.assertAlmostEqual(out['length_m'],50)

    def test_exact_quarter_turn(self):
        out=forward_pose_search((0,0,0),(20,20,math.pi/2),20,lambda p:True,bounds=(-100,-100,100,100))
        self.assertAlmostEqual(out['length_m'],10*math.pi)

    def test_obstacle_detour_has_no_parent_replacement_gaps(self):
        bounds=(-15,-25,65,25);obstacles=[(20,-5,30,5)]
        clear=lambda pp:collision_free(pp,obstacles,bounds,step=.5)
        out=forward_pose_search((0,0,0),(50,0,0),5,clear,bounds=bounds,position_resolution=1,straight_step=2,shot_every=5,max_expansions=10000)
        self.check_path(out,(0,0,0),(50,0,0),5,clear);self.assertGreater(out['length_m'],50)

    def test_budget_failure_is_explicit(self):
        out=forward_pose_search((0,0,0),(50,0,0),5,lambda pp:False,bounds=(-100,-100,100,100),max_expansions=1)
        self.assertFalse(out['passed']);self.assertEqual(out['expanded_states'],1)

    def test_invalid_search_settings(self):
        with self.assertRaises(ValueError):forward_pose_search((0,0,0),(1,0,0),0,lambda p:True,bounds=(-1,-1,2,2))

    def test_goal_shot_interval_one_is_supported(self):
        out=forward_pose_search((0,0,0),(50,0,0),5,lambda p:True,bounds=(-100,-100,100,100),shot_every=1)
        self.assertTrue(out['passed']);self.assertAlmostEqual(out['length_m'],50)


if __name__=='__main__':unittest.main()
