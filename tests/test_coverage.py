import copy
import math
import unittest

from berthing_planner.coverage import append_sweep, plan_coverage
from berthing_planner.occupancy import OccupancyQuadTree


def independent_segment_hits_rectangle(a,b,box):
    """Closed-edge orientation oracle, independent of the tree's slab test."""
    left,bottom,right,top = box
    if any(left<=p[0]<=right and bottom<=p[1]<=top for p in (a,b)):
        return True
    corners = [(left,bottom),(right,bottom),(right,top),(left,top)]
    def cross(p,q,r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    for c,d in zip(corners,corners[1:]+corners[:1]):
        if max(min(a[0],b[0]),min(c[0],d[0]))>min(max(a[0],b[0]),max(c[0],d[0])):
            continue
        if max(min(a[1],b[1]),min(c[1],d[1]))>min(max(a[1],b[1]),max(c[1],d[1])):
            continue
        if cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0:
            return True
    return False


class CoverageTests(unittest.TestCase):
    def check_continuity_and_geometry(self, tree, plan):
        self.assertAlmostEqual(plan['length_m'], sum(math.dist(a,b) for a,b in zip(plan['path'],plan['path'][1:])))
        previous = None
        for leg in plan['legs']:
            self.assertEqual(leg['scanning'], leg['kind'] in ('sweep','supplement'))
            if leg['scanning']:
                self.assertEqual(len(leg['points']), 2)
            if previous is not None:
                self.assertEqual(previous, leg['points'][0])
            self.assertAlmostEqual(leg['length_m'], sum(math.dist(a,b) for a,b in zip(leg['points'],leg['points'][1:])))
            self.assertTrue(all(tree.line_of_sight(a,b) for a,b in zip(leg['points'],leg['points'][1:])))
            self.assertFalse(any(independent_segment_hits_rectangle(a,b,box)
                                 for a,b in zip(leg['points'],leg['points'][1:]) for box in tree.obstacles))
            previous = leg['points'][-1]
        self.assertAlmostEqual(plan['length_m'],plan['scan_length_m']+plan['transit_length_m'])

    def test_hand_checked_empty_rectangle(self):
        rows = [[0]*8 for _ in range(8)]
        tree = OccupancyQuadTree((0,0,80,80), rows)
        plan = plan_coverage(tree,rows,lane_spacing_m=20,lane_offset_m=10)
        self.assertEqual(plan['actual_scan_rows'], [1,3,5,7])
        self.assertEqual([r['actual_y_m'] for r in plan['scan_rows']], [15,35,55,75])
        self.assertEqual(plan['path'][0], [5,15])
        self.assertEqual(plan['path'][-1], [5,75])
        self.assertEqual(plan['scan_length_m'], 280)
        self.assertEqual(plan['transit_length_m'], 60)
        self.assertEqual(plan['length_m'], 340)
        self.assertEqual(plan['scan_leg_count'], 4)
        self.assertEqual(plan['direction_change_count'], 6)
        self.check_continuity_and_geometry(tree,plan)

    def test_hole_splits_runs_and_requires_scan_off_detours(self):
        rows = [[int(2<=j<=5 and 3<=i<=4) for i in range(8)] for j in range(8)]
        tree = OccupancyQuadTree((0,0,80,80), rows)
        plan = plan_coverage(tree,rows,lane_spacing_m=20,lane_offset_m=10)
        self.assertEqual([r['free_run_count'] for r in plan['scan_rows']], [1,2,2,1])
        scans = [leg for leg in plan['legs'] if leg['scanning']]
        self.assertEqual(len(scans),6)
        self.assertEqual(scans[1]['points'], [[75,35],[55,35]])
        self.assertEqual(scans[2]['points'], [[25,35],[5,35]])
        detours = [leg for leg in plan['legs'] if leg.get('connection_method')=='theta_astar_then_LOS_shortcut']
        self.assertEqual(len(detours),2)
        self.assertTrue(all(leg['length_m']>math.dist(leg['points'][0],leg['points'][-1]) for leg in detours))
        self.check_continuity_and_geometry(tree,plan)

    def test_disconnected_scan_runs_raise_instead_of_skipping(self):
        rows = [[int(i==3) for i in range(8)] for _ in range(8)]
        tree = OccupancyQuadTree((0,0,80,80),rows)
        with self.assertRaisesRegex(ValueError,'No path'):
            plan_coverage(tree,rows,lane_spacing_m=20,lane_offset_m=10)

    def test_supplement_adds_continuous_scan_off_connection(self):
        rows = [[0]*8 for _ in range(8)]
        tree = OccupancyQuadTree((0,0,80,80),rows)
        plan = plan_coverage(tree,rows,lane_spacing_m=100,lane_offset_m=10)
        self.assertIs(append_sweep(tree,plan,(5,35),(75,35)),plan)
        self.assertEqual([leg['kind'] for leg in plan['legs']],['sweep','transit','supplement'])
        self.assertAlmostEqual(plan['transit_length_m'],math.hypot(70,20))
        self.assertEqual(plan['supplement_length_m'],70)
        self.assertEqual(plan['scan_leg_count'],2)
        self.check_continuity_and_geometry(tree,plan)

    def test_append_failure_leaves_existing_plan_unchanged(self):
        rows = [[int(i==3) for i in range(8)] for _ in range(8)]
        tree = OccupancyQuadTree((0,0,80,80),rows)
        plan = {'path':[],'legs':[],'length_m':0.0}
        append_sweep(tree,plan,(5,15),(25,15),kind='sweep')
        before = copy.deepcopy(plan)
        with self.assertRaisesRegex(ValueError,'No path'):
            append_sweep(tree,plan,(45,15),(75,15))
        self.assertEqual(plan,before)
        with self.assertRaisesRegex(ValueError,'Scanning leg'):
            append_sweep(tree,plan,(5,35),(75,35))
        self.assertEqual(plan,before)

    def test_nonzero_origin_and_exact_midpoint_snap(self):
        rows = [[0]*8 for _ in range(8)]
        tree = OccupancyQuadTree((100,200,180,280),rows)
        plan = plan_coverage(tree,rows,lane_spacing_m=20,lane_offset_m=0)
        self.assertEqual([r['requested_y_m'] for r in plan['scan_rows']],[200,220,240,260])
        self.assertEqual([r['actual_y_m'] for r in plan['scan_rows']],[205,225,245,265])
        plan = plan_coverage(tree,rows,lane_spacing_m=20,lane_offset_m=10)
        self.assertEqual(plan['scan_rows'][0]['row_index'],1)
        self.assertEqual(plan['scan_rows'][0]['actual_y_m'],215)

    def test_scan_on_closed_obstacle_boundary_is_rejected(self):
        rows = [[int(i==3) for i in range(8)] for _ in range(8)]
        tree = OccupancyQuadTree((0,0,80,80),rows)
        # x=40 belongs to a free cell but touches the wall's closed right edge.
        self.assertFalse(tree.locate((40,5)).blocked)
        plan = {'path':[],'legs':[],'length_m':0.0}
        with self.assertRaisesRegex(ValueError,'Scanning leg'):
            append_sweep(tree,plan,(40,5),(40,75))
        self.assertEqual(plan,{'path':[],'legs':[],'length_m':0.0})

    def test_spacing_phase_and_empty_scan_boundaries(self):
        rows = [[0]*8 for _ in range(8)]
        tree = OccupancyQuadTree((0,0,80,80),rows)
        for spacing,offset in ((0,0),(-1,0),(9,0),(20,-1),(20,20),(math.inf,0),(20,math.nan)):
            with self.subTest(spacing=spacing,offset=offset), self.assertRaises(ValueError):
                plan_coverage(tree,rows,lane_spacing_m=spacing,lane_offset_m=offset)
        with self.assertRaisesRegex(ValueError,'No free scanning run'):
            plan_coverage(tree,rows,lane_spacing_m=200,lane_offset_m=100)
        with self.assertRaisesRegex(ValueError,'dimensions'):
            plan_coverage(tree,rows[:-1],lane_spacing_m=20,lane_offset_m=0)
        blocked = [[1]*8 for _ in range(8)]
        with self.assertRaisesRegex(ValueError,'No free scanning run'):
            plan_coverage(OccupancyQuadTree((0,0,80,80),blocked),blocked,
                          lane_spacing_m=20,lane_offset_m=0)

    def test_zero_length_run_is_retained_without_fake_turns(self):
        rows = [[0]]
        tree = OccupancyQuadTree((0,0,10,10),rows)
        plan = plan_coverage(tree,rows,lane_spacing_m=10,lane_offset_m=5)
        self.assertEqual(plan['legs'][0]['points'],[[5,5],[5,5]])
        self.assertEqual(plan['length_m'],0)
        self.assertEqual(plan['scan_leg_count'],1)
        self.assertEqual(plan['direction_change_count'],0)
        self.check_continuity_and_geometry(tree,plan)

    def test_direction_changes_are_polyline_turns_not_drive_reversals(self):
        tree = OccupancyQuadTree((0,0,80,80),[[0]*8 for _ in range(8)])
        plan = {'path':[],'legs':[],'length_m':0.0}
        append_sweep(tree,plan,(5,5),(25,5),'sweep')
        append_sweep(tree,plan,(25,5),(45,5))
        self.assertEqual(plan['direction_change_count'],0)
        append_sweep(tree,plan,(45,5),(25,5))
        self.assertEqual(plan['direction_change_count'],1)


if __name__=='__main__':
    unittest.main()
