import copy
import math
import unittest

from berthing_planner.cooperative_allocation import allocate_scan_tasks, plan_scan_tasks
from berthing_planner.occupancy import OccupancyQuadTree


def scan(a, b, kind='sweep'):
    return {'kind': kind, 'scanning': True, 'points': [list(a),list(b)], 'length_m': math.dist(a,b)}


def task(a, b, identifier, index=0, kind='sweep'):
    return {**{k:v for k,v in scan(a,b,kind).items() if k!='scanning'},
            'task_id':identifier, 'source_leg_index':index}


class CooperativeAllocationTests(unittest.TestCase):
    def setUp(self):
        self.tree = OccupancyQuadTree((0,0,80,80),[[0]*8 for _ in range(8)])

    def check_accounting_and_geometry(self, result):
        tasks = {t['task_id']:t for t in result['tasks']}
        identifiers = []
        for platform in result['platforms'].values():
            active = [leg for leg in platform['legs'] if leg['scanning']]
            self.assertEqual(platform['task_ids'],[leg['task_id'] for leg in active])
            for leg in active:
                source = tasks[leg['task_id']]
                self.assertIn(leg['points'],[source['points'],source['points'][::-1]])
                self.assertEqual(leg['kind'],source['kind'])
                self.assertAlmostEqual(leg['length_m'],source['length_m'])
                identifiers.append(leg['task_id'])
            self.assertTrue(all(not l['scanning'] for l in platform['legs'] if l['kind']=='transit'))
            self.assertTrue(all(self.tree.line_of_sight(a,b)
                                for a,b in zip(platform['path'],platform['path'][1:])))
            self.assertEqual(platform['deployed_start'],platform['path'][0])
            self.assertEqual(platform['route_end'],platform['path'][-1])
            self.assertAlmostEqual(platform['length_m'],sum(math.dist(a,b) for a,b in zip(platform['path'],platform['path'][1:])))
        self.assertEqual(sorted(identifiers),sorted(tasks))
        self.assertEqual(len(identifiers),len(set(identifiers)))

    def test_hand_checked_two_balanced_bands(self):
        legs = [scan((5,y),(75,y)) for y in (15,35,55,75)]
        before = copy.deepcopy(legs)
        result = allocate_scan_tasks(self.tree,legs)
        self.assertEqual(result['allocation']['selected_cut_index'],2)
        self.assertEqual(result['allocation']['split_y_m'],45)
        self.assertEqual(result['allocation']['estimated_route_lengths_m'],{'uuv_1':160,'uuv_2':160})
        self.assertEqual(result['allocation']['actual_route_lengths_m'],{'uuv_1':160,'uuv_2':160})
        self.assertEqual(result['platforms']['uuv_1']['task_ids'],['scan_000','scan_001'])
        self.assertEqual(result['platforms']['uuv_2']['task_ids'],['scan_002','scan_003'])
        self.assertFalse(result['allocation']['optimality_claimed'])
        self.assertFalse(result['allocation']['launch_and_recovery_included'])
        self.assertEqual(legs,before)
        self.check_accounting_and_geometry(result)
        control = plan_scan_tasks(self.tree,result['tasks'])
        self.assertEqual(control['length_m'],340)

    def test_task_ids_follow_source_scan_order_and_transits_are_rebuilt(self):
        legs = [scan((5,15),(75,15)),
                {'kind':'transit','scanning':False,'points':[[75,15],[75,35]],'length_m':20},
                scan((75,35),(5,35)),scan((5,75),(75,75)),
                scan((25,55),(45,55),'supplement')]
        result = allocate_scan_tasks(self.tree,legs)
        self.assertEqual([t['source_leg_index'] for t in result['tasks']],[0,2,3,4])
        self.assertEqual(result['tasks'][-1]['kind'],'supplement')
        control = plan_scan_tasks(self.tree,result['tasks'])
        self.assertEqual(control['task_ids'],['scan_000','scan_001','scan_003','scan_002'])
        self.check_accounting_and_geometry(result)

    def test_equal_y_groups_stay_together_when_a_group_boundary_exists(self):
        legs = [scan((5,15),(25,15)),scan((35,15),(55,15)),scan((65,15),(75,15)),
                scan((5,65),(75,65))]
        result = allocate_scan_tasks(self.tree,legs)
        self.assertTrue(result['allocation']['equal_y_groups_preserved'])
        self.assertEqual(result['allocation']['selected_cut_index'],3)
        self.assertEqual(len(result['platforms']['uuv_1']['task_ids']),3)
        self.check_accounting_and_geometry(result)

    def test_same_y_only_fallback_is_explicit_and_deterministic(self):
        legs = [scan((x,15),(x+10,15)) for x in (5,25,45,65)]
        first = allocate_scan_tasks(self.tree,legs,platform_ids=('south','north'))
        second = allocate_scan_tasks(self.tree,legs,platform_ids=('south','north'))
        self.assertEqual(first,second)
        self.assertFalse(first['allocation']['equal_y_groups_preserved'])
        self.assertTrue(first['allocation']['same_y_split_required'])
        self.check_accounting_and_geometry(first)

    def test_control_uses_safe_astar_around_obstacle_and_no_hidden_deployment(self):
        rows = [[int(2<=j<=5 and 3<=i<=4) for i in range(8)] for j in range(8)]
        tree = OccupancyQuadTree((0,0,80,80),rows)
        tasks = [task((5,35),(25,35),'left'),task((55,35),(75,35),'right',1)]
        plan = plan_scan_tasks(tree,tasks)
        self.assertEqual([leg['kind'] for leg in plan['legs']],['sweep','transit','sweep'])
        self.assertEqual(plan['legs'][1]['connection_method'],'theta_astar_then_LOS_shortcut')
        self.assertGreater(plan['transit_length_m'],30)
        self.assertEqual(plan['path'][0],[5,35])
        self.assertEqual(plan['path'][-1],[75,35])
        self.assertTrue(all(tree.line_of_sight(a,b) for a,b in zip(plan['path'],plan['path'][1:])))

    def test_no_path_is_reported_instead_of_skipping_tasks(self):
        rows = [[int(i==3) for i in range(8)] for _ in range(8)]
        tree = OccupancyQuadTree((0,0,80,80),rows)
        tasks = [task((5,35),(25,35),'left'),task((45,35),(75,35),'right',1)]
        with self.assertRaisesRegex(ValueError,'No path'):
            plan_scan_tasks(tree,tasks)
        legs = [scan((5,15),(25,15)),scan((45,15),(75,15)),scan((5,65),(25,65))]
        with self.assertRaisesRegex(ValueError,'No path'):
            allocate_scan_tasks(tree,legs)

    def test_zero_length_active_tasks_are_not_dropped(self):
        result = allocate_scan_tasks(self.tree,[scan((5,15),(5,15)),scan((5,35),(5,35),'supplement')])
        self.assertEqual(result['allocation']['task_count'],2)
        self.assertEqual(result['allocation']['scan_length_m'],0)
        self.assertEqual(result['allocation']['actual_length_imbalance_fraction'],0)
        self.check_accounting_and_geometry(result)

    def test_invalid_source_scan_flags_geometry_length_and_ids_are_rejected(self):
        valid = [scan((5,15),(75,15)),scan((5,35),(75,35))]
        for change in (
                {'kind':'transit','scanning':True}, {'scanning':False},
                {'points':[[5,15],[35,15],[75,15]]}, {'length_m':1},
                {'points':[[math.nan,15],[75,15]]}, {'kind':'curve'}):
            bad = copy.deepcopy(valid);bad[0].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                allocate_scan_tasks(self.tree,bad)
        with self.assertRaises(ValueError):
            allocate_scan_tasks(self.tree,valid[:1])
        for ids in (('one',),('same','same'),('one','two','three'),('one','')):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                allocate_scan_tasks(self.tree,valid,ids)
        duplicated = [task((5,15),(75,15),'same'),task((5,35),(75,35),'same',1)]
        with self.assertRaisesRegex(ValueError,'unique'):
            plan_scan_tasks(self.tree,duplicated)

    def test_empty_single_control_is_explicitly_idle(self):
        plan = plan_scan_tasks(self.tree,[])
        self.assertEqual(plan['path'],[])
        self.assertEqual(plan['legs'],[])
        self.assertEqual(plan['task_ids'],[])
        self.assertEqual(plan['length_m'],0)
        self.assertIsNone(plan['deployed_start'])


if __name__=='__main__':
    unittest.main()
