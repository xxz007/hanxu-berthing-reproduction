"""Run with the existing research workbench environment, not the stdlib-only venv."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('map_reconstruction',Path(__file__).resolve().parents[1]/'scripts/reconstruct_paper_map.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ReconstructionTests(unittest.TestCase):
    def test_horizontal_and_vertical_tick_fit_known_scale(self):
        for ticks,origin,sign in [([[10,0],[110,1000],[210,2000]],10,1),([[210,0],[110,1000],[10,2000]],210,-1)]:
            fit=module.calibrate_ticks(ticks,origin,sign)
            self.assertEqual(fit['meters_per_pixel'],10)
            self.assertEqual(fit['max_abs_residual_m'],0)

    def test_top_image_row_becomes_north_grid_row(self):
        mask=np.zeros((8,8),dtype=bool)
        mask[0,0]=True
        raster=module.rasterize_ink(mask,2)
        self.assertTrue(raster[3,0])
        self.assertEqual(int(raster.sum()),1)

    def test_quadtree_known_single_obstacle_partition(self):
        raster=np.zeros((8,8),dtype=bool)
        raster[3,3]=True
        leaves=module.quadtree_leaves(raster)
        self.assertEqual(len(leaves),10)
        self.assertEqual(sum(l['blocked'] for l in leaves),1)
        coverage=np.zeros((8,8),dtype=int)
        for l in leaves:
            coverage[l['j0']:l['j1'],l['i0']:l['i1']]+=1
        self.assertTrue(np.all(coverage==1))

    def test_connectivity_does_not_cross_closed_wall(self):
        raster=np.zeros((4,4),dtype=bool)
        raster[:,2]=True
        positions=[{'name':'start','chart_EN_proxy':[.5,.5],'thesis_xy':[.5,.5]},
                   {'name':'transit_goal','chart_EN_proxy':[3.5,3.5],'thesis_xy':[3.5,3.5]}]
        result=module.connectivity(raster,positions,[0,0,4,4])
        self.assertFalse(result['positions']['transit_goal']['blocked'])
        self.assertFalse(result['positions']['transit_goal']['reachable_from_start'])

    def test_blocked_endpoint_is_reported_without_moving_it(self):
        raster=np.zeros((4,4),dtype=bool)
        raster[3,3]=True
        positions=[{'name':'start','chart_EN_proxy':[.5,.5],'thesis_xy':[.5,.5]},
                   {'name':'transit_goal','chart_EN_proxy':[3.5,3.5],'thesis_xy':[3.5,3.5]}]
        result=module.connectivity(raster,positions,[0,0,4,4])
        self.assertTrue(result['positions']['transit_goal']['blocked'])
        self.assertFalse(result['positions']['transit_goal']['reachable_from_start'])
        self.assertEqual(result['positions']['transit_goal']['chart_EN_proxy'],[3.5,3.5])


if __name__=='__main__':
    unittest.main()
