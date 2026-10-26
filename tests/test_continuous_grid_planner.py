from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from local_depth_grid import GridConfig,clearance_mask
from continuous_depth_path import validate_route
from continuous_grid_planner import center_mask,plan_route,shortcut_route


class ContinuousGridPlannerTests(unittest.TestCase):
    def setUp(self):
        self.config=GridConfig();self.states=np.zeros(self.config.shape,dtype=np.int8)

    def grid(self):
        return dict(config=self.config,states=self.states,free=clearance_mask(self.states,.25,.34))

    def test_minimal_unknown_corner_legacy_rejected_square_approved(self):
        self.states[12,28]=-1
        old=plan_route(self.grid(),(0,0),(2,2.5),'legacy_centers')
        new=plan_route(self.grid(),(0,0),(2,2.5),'continuous_square')
        self.assertEqual(old['continuous_gate']['status'],'segment_footprint_blocked')
        self.assertEqual(new['continuous_gate']['status'],'approved')
        self.assertEqual(self.states[12,28],-1)

    def test_every_mask_center_matches_independent_continuous_point_gate(self):
        self.config=GridConfig(right_min_m=-2,right_max_m=2,forward_min_m=-2,forward_max_m=2)
        self.states=np.zeros(self.config.shape,dtype=np.int8)
        for cell,state in [((4,4),-1),((9,10),1),((11,5),2)]:self.states[cell]=state
        before=self.states.copy();mask=center_mask(self.states,self.config)
        for row in range(mask.shape[0]):
            for col in range(mask.shape[1]):
                p=self.config.center((row,col));gate=validate_route(self.states,self.config,[p],start=p,goal=p)
                self.assertEqual(bool(mask[row,col]),gate['status']=='approved',(row,col))
        np.testing.assert_array_equal(before,self.states)

    def test_straight_shortcut_has_only_exact_query_endpoints(self):
        result=plan_route(self.grid(),(0,0),(0,3),'continuous_square_shortcuts')
        self.assertEqual(result['continuous_gate']['waypoints_right_forward_m'],[[0,0],[0,3]])
        self.assertAlmostEqual(result['continuous_gate']['continuous_route_length_m'],3)
        self.assertGreater(result['shortcut_queries'],0)

    def test_barrier_keeps_a_bend_and_original_gate(self):
        self.states[12,20]=2
        result=plan_route(self.grid(),(0,0),(0,3),'continuous_square_shortcuts')
        gate=result['continuous_gate'];self.assertEqual(gate['status'],'approved')
        self.assertGreater(len(gate['waypoints_right_forward_m']),2)
        self.assertEqual(validate_route(self.states,self.config,gate['waypoints_right_forward_m'],start=(0,0),goal=(0,3))['status'],'approved')

    def test_budget_falls_back_to_remaining_already_safe_route(self):
        self.states[12,20]=-1
        route=plan_route(self.grid(),(0,0),(0,3),'continuous_square')['continuous_gate']['waypoints_right_forward_m']
        first=shortcut_route(self.states,self.config,route,max_queries=1)
        self.assertEqual(first,shortcut_route(self.states,self.config,route,max_queries=1))
        self.assertEqual(first['queries'],1);self.assertTrue(first['budget_exhausted'])
        self.assertEqual(validate_route(self.states,self.config,first['route'],start=(0,0),goal=(0,3))['status'],'approved')

    def test_invalid_and_unsafe_shortcuts_rejected(self):
        self.states[12,20]=-1
        with self.assertRaises(ValueError):shortcut_route(self.states,self.config,[[0,0],[0,3]])
        with self.assertRaises(ValueError):plan_route(self.grid(),(0,0),(0,3),'invented')
        with self.assertRaises(ValueError):center_mask(self.states,self.config,tracking_margin_m=-1)

    def test_bounds_and_unknown_remain_blocked(self):
        self.states[self.config.cell(0,3)]=-1
        result=plan_route(self.grid(),(0,0),(0,3),'continuous_square_shortcuts')
        self.assertEqual(result['planning']['status'],'goal_unobserved')
        self.assertEqual(result['continuous_gate']['waypoints_right_forward_m'],[])
        self.assertFalse(center_mask(self.states,self.config)[0,0])
