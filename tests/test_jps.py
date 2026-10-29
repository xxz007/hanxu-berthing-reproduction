import heapq
import itertools
import math
import random
import unittest
from berthing_planner.jps import SquareGrid,search_grid


def oracle(rows,start,goal):
    """Independent Dijkstra: direct occupancy/side checks, no JPS helpers."""
    nx,ny=len(rows[0]),len(rows)
    costs={start:0};queue=[(0,start)]
    while queue:
        cost,(x,y)=heapq.heappop(queue)
        if cost!=costs[(x,y)]:continue
        if (x,y)==goal:return cost
        for dx,dy in itertools.product((-1,0,1),repeat=2):
            a,b=x+dx,y+dy
            if not (dx or dy) or not(0<=a<nx and 0<=b<ny) or rows[b][a]:continue
            if dx and dy and (rows[y][a] or rows[b][x]):continue
            new=cost+math.hypot(dx,dy)
            if new<costs.get((a,b),math.inf):
                costs[(a,b)]=new;heapq.heappush(queue,(new,(a,b)))
    return None


class JPSTests(unittest.TestCase):
    def check_case(self,rows,start,goal):
        expected=oracle(rows,start,goal)
        grid=SquareGrid(rows)
        for algorithm in ('jps','astar'):
            r=search_grid(grid,start,goal,algorithm)
            if expected is None:
                self.assertEqual(r['status'],'no_path',(rows,start,goal,algorithm))
            else:
                self.assertEqual(r['status'],'path_found',(rows,start,goal,algorithm))
                self.assertAlmostEqual(r['grid_length_m'],expected,places=9,msg=str((rows,start,goal,algorithm)))
                for a,b in zip(r['cell_path'],r['cell_path'][1:]):
                    dx,dy=b[0]-a[0],b[1]-a[1]
                    self.assertTrue(dx==0 or dy==0 or abs(dx)==abs(dy))
                    sx,sy=(dx>0)-(dx<0),(dy>0)-(dy<0)
                    for i in range(max(abs(dx),abs(dy))):
                        self.assertTrue(grid.can_step(a[0]+i*sx,a[1]+i*sy,sx,sy))

    def test_all_three_by_three_maps_all_free_endpoint_pairs(self):
        for mask in range(1<<9):
            rows=[[int(bool(mask&(1<<(3*y+x)))) for x in range(3)] for y in range(3)]
            free=[(x,y) for y in range(3) for x in range(3) if not rows[y][x]]
            for i,start in enumerate(free):
                for goal in free[i:]:self.check_case(rows,start,goal)

    def test_random_larger_maps_against_independent_dijkstra(self):
        rng=random.Random(20261029)
        for _ in range(250):
            rows=[[int(rng.random()<.25) for x in range(12)] for y in range(10)]
            start=(rng.randrange(12),rng.randrange(10));goal=(rng.randrange(12),rng.randrange(10))
            rows[start[1]][start[0]]=rows[goal[1]][goal[0]]=0
            self.check_case(rows,start,goal)

    def test_all_four_by_four_obstacle_maps_between_opposite_corners(self):
        for mask in range(1<<14):
            cells=[0]+[int(bool(mask&(1<<i))) for i in range(14)]+[0]
            self.check_case([cells[i:i+4] for i in range(0,16,4)],(0,0),(3,3))

    def test_no_corner_cutting_and_open_map_pruning(self):
        self.check_case([[0,1],[1,0]],(0,0),(1,1))
        grid=SquareGrid([[0]*40 for _ in range(30)])
        a=search_grid(grid,(0,0),(39,29),'astar');j=search_grid(grid,(0,0),(39,29),'jps')
        self.assertAlmostEqual(a['grid_length_m'],j['grid_length_m'])
        self.assertLess(j['closed_unique'],a['closed_unique'])
        self.assertGreater(j['jump_scan_steps'],j['closed_unique'])

    def test_resampling_closed_map_uses_conservative_occupancy_and_padding(self):
        grid=SquareGrid.from_rectangles((0,0,25,25),[(11,11,12,12)],10)
        self.assertEqual((grid.nx,grid.ny),(3,3))
        self.assertFalse(grid.free(1,1))
        self.assertFalse(grid.free(2,0))
        self.assertFalse(grid.free(0,2))
        self.assertEqual(grid.locate((5,5)),(0,0))
        with self.assertRaises(ValueError):grid.locate((24,5))


if __name__=='__main__':
    unittest.main()
