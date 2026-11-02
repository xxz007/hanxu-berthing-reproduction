import math
import unittest
import numpy as np
from chart_geometry import Geos
from navigation_geometry import NavigationGeometry


class StaticNavigationTests(unittest.TestCase):
    def setUp(self):
        self.nav={'type':'Polygon','coordinates':[[[0,0],[100,0],[100,100],[0,100],[0,0]],[[40,40],[40,60],[60,60],[60,40],[40,40]]]}
        self.hazards={'type':'Point','coordinates':[50,50]}

    def test_static_queries_match_existing_native_boundary(self):
        rng=np.random.default_rng(573)
        with NavigationGeometry(self.nav,self.hazards)as n,Geos()as g:
            for xy in rng.uniform(-10,110,(100,2)):
                shape={'type':'Point','coordinates':xy.tolist()}
                self.assertEqual(n.covers(shape),g.covers(self.nav,shape))
                self.assertAlmostEqual(n.distance(shape),g.distance(self.nav,shape))
                self.assertAlmostEqual(n.distance(shape,to_hazards=True),g.distance(self.hazards,shape))

    def test_hole_crossing_rejected_even_with_free_endpoints(self):
        with NavigationGeometry(self.nav,self.hazards)as n:
            self.assertTrue(n.covers({'type':'Point','coordinates':[20,50]}))
            self.assertTrue(n.covers({'type':'Point','coordinates':[80,50]}))
            self.assertFalse(n.covers({'type':'LineString','coordinates':[[20,50],[80,50]]}))

    def test_closed_boundary_and_positive_outside_distance(self):
        with NavigationGeometry(self.nav,self.hazards)as n:
            self.assertTrue(n.covers({'type':'Point','coordinates':[0,50]}))
            self.assertFalse(n.covers({'type':'Point','coordinates':[-.001,50]}))
            self.assertAlmostEqual(n.distance({'type':'Point','coordinates':[-.001,50]}),.001)

    def test_invalid_geojson_is_explicit_failure(self):
        with NavigationGeometry(self.nav,self.hazards)as n:
            with self.assertRaises(ValueError):n.covers({'type':'NoSuchGeometry','coordinates':[]})


if __name__=='__main__':unittest.main()
