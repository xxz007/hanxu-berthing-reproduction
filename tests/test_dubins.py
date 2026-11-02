import math
import random
import unittest
from berthing_planner.dubins import candidates


class DubinsTests(unittest.TestCase):
    def test_straight_known_solution(self):
        self.assertAlmostEqual(candidates((0,0,0),(100,0,0),20)[0]['length_m'],100)

    def test_quarter_circle_known_solution(self):
        self.assertAlmostEqual(candidates((0,0,0),(20,20,math.pi/2),20)[0]['length_m'],10*math.pi)

    def test_identity_has_no_fake_motion(self):
        self.assertEqual(candidates((0,0,0),(0,0,2*math.pi),20)[0]['pieces'],[])

    def test_same_position_opposite_heading_is_not_identity(self):
        self.assertGreater(candidates((0,0,0),(0,0,math.pi),20)[0]['length_m'],0)

    def test_random_endpoint_radius_and_heading(self):
        rng=random.Random(573)
        for _ in range(300):
            start=(rng.uniform(-100,100),rng.uniform(-100,100),rng.uniform(-math.pi,math.pi))
            goal=(rng.uniform(-100,100),rng.uniform(-100,100),rng.uniform(-math.pi,math.pi));radius=rng.uniform(5,60)
            paths=candidates(start,goal,radius);self.assertTrue(paths)
            for path in paths:
                pose=start
                for p in path['pieces']:
                    self.assertEqual(p.direction,1);self.assertLessEqual(abs(p.curvature),1/radius+1e-12)
                    self.assertLess(math.dist(pose[:2],p.pose[:2]),1e-7);pose=p.evaluate(1)[:3]
                self.assertLess(math.dist(pose[:2],goal[:2]),1e-5)
                self.assertLess(abs(math.remainder(pose[2]-goal[2],2*math.pi)),1e-7)
                self.assertGreaterEqual(path['length_m']+1e-6,math.dist(start[:2],goal[:2]))

    def test_invalid_radius_and_pose(self):
        for r in (0,-1,math.nan):
            with self.assertRaises(ValueError):candidates((0,0,0),(1,1,0),r)


if __name__=='__main__':unittest.main()
