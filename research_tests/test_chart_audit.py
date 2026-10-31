"""Hand-checkable boundary cases for the independent chart audit."""
import unittest
import numpy as np
from audit_open_chart_stage_a import inside


class ChartAuditTests(unittest.TestCase):
    def test_closed_polygon_and_hole_boundary(self):
        polygon={'type':'Polygon','coordinates':[
            [[0,0],[4,0],[4,4],[0,4],[0,0]],
            [[1,1],[3,1],[3,3],[1,3],[1,1]]]}
        points=np.array([[0,0],[2,2],[1,2],[.5,.5],[4.000001,2],[4,2]])
        contained,boundary=inside(polygon,points)
        np.testing.assert_array_equal(contained,[True,False,True,True,False,True])
        np.testing.assert_array_equal(boundary,[True,False,True,False,False,True])

    def test_concave_task_diagonal_boundary(self):
        polygon={'type':'Polygon','coordinates':[[
            [750,1100],[4350,900],[5050,2450],[3950,3550],
            [2550,3350],[1450,4250],[800,3100],[750,1100]]]}
        points=np.array([[1835,3935],[1725,4025],[2865,3395],[3000,2000],[600,1000]])
        contained,boundary=inside(polygon,points)
        np.testing.assert_array_equal(contained,[True,True,True,True,False])
        np.testing.assert_array_equal(boundary,[True,True,True,False,False])


if __name__=='__main__':
    unittest.main()
