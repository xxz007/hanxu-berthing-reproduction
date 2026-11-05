import math
import unittest
from berthing_planner.tracking import feedback_command, plant_step


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.limits = dict(speed_mps=2, acceleration_mps2=.2, yaw_rate_radps=.15, yaw_acceleration_radps2=.05)
        self.gains = dict(position_p=.25, velocity_d=1, position_i=.02, integral_limit=2, heading_p=1, yaw_rate_d=1.5)

    def test_plant_exact_quadratic_with_drift(self):
        actual = plant_step((1,2,.3,2,3,.1),(.1,-.2,.02),(.4,-.5),2)
        for a, b in zip(actual, (6,6.6,.54,2.2,2.6,.14)):
            self.assertAlmostEqual(a, b)

    def test_plant_does_not_reset_to_reference(self):
        self.assertEqual(plant_step((10,20,0,0,0,0),(0,0,0),(1,0),2)[:2], (12,20))

    def test_saturation_and_speed_cap(self):
        state=(0,0,0,1.99,0,.149)
        cmd,_=feedback_command(state,state,(100,100,2,4,4,1),(2,2,1),(0,0),.5,self.limits,self.gains)
        out=plant_step(state,cmd,(0,0),.5)
        self.assertLessEqual(math.hypot(*cmd[:2]),.2+1e-12)
        self.assertLessEqual(math.hypot(*out[3:5]),2+1e-12)
        self.assertLessEqual(abs(out[5]),.15+1e-12)
        self.assertLessEqual(abs(cmd[2]),.05+1e-12)

    def test_feedback_rejects_initial_position_offset(self):
        state=(1,-1,.2,0,0,0); integ=(0,0)
        for _ in range(800):
            cmd,integ=feedback_command(state,state,(0,0,0,0,0,0),(0,0,0),integ,.1,self.limits,self.gains)
            state=plant_step(state,cmd,(0,0),.1)
        self.assertLess(math.hypot(*state[:2]),.08)
        self.assertLess(abs(state[2]),.001)

    def test_angle_wrap_uses_short_direction(self):
        state=(0,0,math.pi-.01,0,0,0)
        cmd,_=feedback_command(state,state,(0,0,-math.pi+.01,0,0,0),(0,0,0),(0,0),.1,self.limits,self.gains)
        self.assertGreater(cmd[2],0)


if __name__ == '__main__':unittest.main()
