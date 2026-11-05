import unittest
import numpy as np
from audit_tracking_control import plant_residuals,enclosure_error


class ActualTrajectoryAuditTests(unittest.TestCase):
    def fixture(self):
        t=np.array([0.,.5,1.]);s=np.zeros((3,3,6));u=np.zeros((2,3,3));u[:,:,0]=.2
        s[:,:,0]=.1*t[:,None]**2+.3*t[:,None];s[:,:,3]=.2*t[:,None]
        return t,s,u,np.tile([.3,0.],(3,1))

    def test_known_constant_acceleration_and_drift(self):
        self.assertLess(max(plant_residuals(*self.fixture()).values()),1e-12)

    def test_position_reset_cannot_pass_plant_equations(self):
        t,s,u,d=self.fixture();s[1,0,0]+=1
        self.assertGreater(plant_residuals(t,s,u,d)['position_integral_error_m'],.9)

    def test_unreported_actuation_cannot_match_velocity(self):
        t,s,u,d=self.fixture();u[0,1,0]=0
        self.assertGreater(plant_residuals(t,s,u,d)['velocity_integral_error'],.09)

    def test_path_enclosure_uses_all_intermediate_samples(self):
        xy=np.array([[0,0],[1,2],[2,0]],float)
        self.assertEqual(enclosure_error(xy,[0,2]),2)
        with self.assertRaises(ValueError):enclosure_error(xy,[1,2])


if __name__=='__main__':unittest.main()
