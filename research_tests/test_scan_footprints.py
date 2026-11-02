import math
import unittest
import numpy as np
from chart_geometry import Geos
from scan_footprints import straight_footprint_bounds


class StraightFootprintBoundsTests(unittest.TestCase):
    def test_exact_capsule_area_is_bracketed(self):
        with Geos()as g:
            lower,upper=straight_footprint_bounds(g,[[0,0],[100,0]],50)
            exact=100*100+math.pi*50**2
            self.assertLess(g.measure(lower)['area_m2'],exact)
            self.assertGreater(g.measure(upper)['area_m2'],exact)

    def test_long_strip_is_not_artificially_widened(self):
        excess=[]
        with Geos()as g:
            for length in (1000,100000):
                _,upper=straight_footprint_bounds(g,[[0,0],[length,0]],50)
                excess.append(g.measure(upper)['area_m2']-(2*50*length+math.pi*50**2))
        # Only the declared1e-6m guard widens the strip, rather than the much
        # larger circumscribed-disk radius correction along its full length.
        self.assertAlmostEqual(excess[1]-excess[0],2e-6*(100000-1000),delta=1e-5)
        self.assertLess(excess[0],5)

    def test_rotated_exact_circle_boundary_samples_inside_upper(self):
        a=np.array([13.,17.]);b=np.array([83.,99.]);u=(b-a)/np.linalg.norm(b-a);normal=np.array([-u[1],u[0]])*50
        theta=np.linspace(0,2*math.pi,1441);circle=np.column_stack((np.cos(theta),np.sin(theta)))*50
        samples=np.vstack((a+circle,b+circle,a+normal,b+normal,a-normal,b-normal))
        with Geos()as g:
            _,upper=straight_footprint_bounds(g,[a.tolist(),b.tolist()],50)
            # Complete endpoint disks and strip are included, so whole circular
            # cross-sections are a stronger check than only two cap endpoints.
            self.assertTrue(g.covers(upper,{'type':'MultiPoint','coordinates':samples.tolist()}))


if __name__=='__main__':unittest.main()
