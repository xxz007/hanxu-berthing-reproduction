import math
import unittest
from berthing_planner.stern_axis import connect_stern_axis
from berthing_planner.curves import wrap


class SternAxisTests(unittest.TestCase):
    def test_known_quarter_turn_and_reverse_axis(self):
        r=connect_stern_axis((2000,0,math.pi/2),(0,1000,0),.005,[],(-500,-500,3000,2000),[200])
        self.assertIsNotNone(r)
        self.assertEqual(r['modes'],'SRS')
        self.assertAlmostEqual(r['signed_lengths_m'][0],800)
        self.assertAlmostEqual(r['signed_lengths_m'][1],100*math.pi)
        self.assertAlmostEqual(r['signed_lengths_m'][2],-2200)
        self.assertEqual(r['pieces'][-1].direction,-1)

    def test_rotated_translated_and_reflected_endpoints(self):
        for rotation in (.3,-1.4):
            for sign in (-1,1):
                c,s=math.cos(rotation),math.sin(rotation)
                start=(100+2000*c,-400+2000*s,sign*math.pi/2+rotation)
                goal=(100-sign*1000*s,-400+sign*1000*c,rotation)
                r=connect_stern_axis(start,goal,.005,[],(-10000,-10000,10000,10000),[200,300])
                self.assertIsNotNone(r)
                p=r['pieces'][-1].evaluate(1)
                self.assertLess(math.dist(p[:2],goal[:2]),1e-8)
                self.assertLess(abs(wrap(p[2]-goal[2])),1e-10)

    def test_collision_and_invalid_radius(self):
        self.assertIsNone(connect_stern_axis((2000,0,math.pi/2),(0,1000,0),.005,
            [(1000,990,1100,1010)],(-500,-500,3000,2000),[200]))
        with self.assertRaises(ValueError):
            connect_stern_axis((2000,0,math.pi/2),(0,1000,0),.005,[],(-500,-500,3000,2000),[199])


if __name__=='__main__':
    unittest.main()
