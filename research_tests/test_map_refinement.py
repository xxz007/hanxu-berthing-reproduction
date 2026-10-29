import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from refine_paper_map import thin_zhang_suen,topology


class RefinementTests(unittest.TestCase):
    def test_single_pixel_and_one_pixel_line_unchanged(self):
        for shape in (([4],[4]),(list(range(2,7)),[4]*5)):
            a=np.zeros((9,9),dtype=bool)
            a[shape]=True
            result,_=thin_zhang_suen(a)
            self.assertTrue(np.array_equal(a,result))

    def test_filled_square_reduces_to_center_pixel(self):
        a=np.zeros((9,9),dtype=bool);a[3:6,3:6]=True
        result,_=thin_zhang_suen(a)
        self.assertEqual(int(result.sum()),1)
        self.assertTrue(result[4,4])

    def test_closed_ring_retains_hole(self):
        a=np.zeros((21,21),dtype=bool);a[3:18,3:18]=True;a[7:14,7:14]=False
        result,_=thin_zhang_suen(a)
        self.assertEqual(topology(a)['enclosed_background_holes_4'],1)
        self.assertEqual(topology(result)['enclosed_background_holes_4'],1)
        self.assertTrue(np.all(~result|a))
        self.assertLess(int(result.sum()),int(a.sum()))

    def test_separate_objects_not_connected(self):
        a=np.zeros((20,20),dtype=bool);a[2:6,2:17]=True;a[12:16,2:17]=True
        result,_=thin_zhang_suen(a)
        self.assertEqual(topology(result)['foreground_components_8'],2)


if __name__=='__main__':
    unittest.main()
