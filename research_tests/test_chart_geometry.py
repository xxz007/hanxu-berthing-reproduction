"""Known-area and topology checks for the native preprocessing boundary."""
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from chart_geometry import Geos


def rectangle(x0,y0,x1,y1):
    return {'type':'Polygon','coordinates':[[[x0,y0],[x1,y0],[x1,y1],[x0,y1],[x0,y0]]]}


class ChartGeometryTests(unittest.TestCase):
    def test_known_overlap_and_area_partition(self):
        a,b=rectangle(0,0,2,2),rectangle(1,1,3,3)
        with Geos() as g:
            union=g.operation('union',{'type':'GeometryCollection','geometries':[a,b]})
            intersection=g.operation('intersection',a,b)
            difference=g.operation('difference',a,b)
            self.assertAlmostEqual(g.measure(union)['area_m2'],7)
            self.assertAlmostEqual(g.measure(intersection)['area_m2'],1)
            self.assertAlmostEqual(g.measure(difference)['area_m2'],3)

    def test_holes_and_empty_exclusion_are_preserved(self):
        outer,inner=rectangle(0,0,4,4),rectangle(1,1,3,3)
        with Geos() as g:
            holed=g.operation('difference',outer,inner)
            self.assertAlmostEqual(g.measure(holed)['area_m2'],12)
            self.assertAlmostEqual(g.measure(g.operation('intersection',holed,inner))['area_m2'],0)
            unchanged=g.operation('difference',holed,{'type':'GeometryCollection','geometries':[]})
            self.assertAlmostEqual(g.measure(unchanged)['area_m2'],12)
            self.assertEqual(len(unchanged['coordinates']),2)

    def test_buffer_chords_stay_outside_required_point_clearance(self):
        radius=50.5/math.cos(math.pi/120)
        with Geos() as g:
            buffered=g.operation('buffer',{'type':'Point','coordinates':[0,0]},distance=radius)
        ring=buffered['coordinates'][0]
        for a,b in zip(ring,ring[1:]):
            dx,dy=b[0]-a[0],b[1]-a[1]
            distance=abs(a[0]*b[1]-a[1]*b[0])/math.hypot(dx,dy)
            self.assertGreaterEqual(distance,50.5-1e-8)

    def test_self_intersection_is_reported_without_repair(self):
        bowtie={'type':'Polygon','coordinates':[[[0,0],[2,2],[0,2],[2,0],[0,0]]]}
        with Geos() as g:
            self.assertFalse(g.measure(bowtie)['valid'])


if __name__=='__main__':
    unittest.main()
