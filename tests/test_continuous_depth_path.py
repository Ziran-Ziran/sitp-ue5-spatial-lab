from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from local_depth_grid import GridConfig,clearance_mask
from continuous_depth_path import validate_route


class ContinuousDepthPathTests(unittest.TestCase):
    def setUp(self):self.config=GridConfig();self.states=np.zeros(self.config.shape,dtype=np.int8)

    def test_clear_floor_includes_actual_query_connectors(self):
        result=validate_route(self.states,self.config,[[.125,.125],[.125,3.125]],start=(0,0),goal=(0,3))
        self.assertEqual(result['status'],'approved')
        self.assertEqual(result['waypoints_right_forward_m'][0],[0,0])
        self.assertEqual(result['waypoints_right_forward_m'][-1],[0,3])

    def test_center_clearance_does_not_certify_actual_start_connector(self):
        self.states[self.config.cell(-.375,-.125)]=-1
        self.assertTrue(clearance_mask(self.states,.25,.34)[self.config.cell(0,0)])
        result=validate_route(self.states,self.config,[[.125,.125],[.125,3.125]],
            start=(0,0),goal=(0,3),tracking_margin_m=0)
        self.assertEqual(result['status'],'start_footprint_blocked')

    def test_unknown_barrier_blocks_segment_even_when_endpoints_free(self):
        self.states[self.config.cell(0,1.5)]=-1
        result=validate_route(self.states,self.config,[[.125,.125],[.125,3.125]],start=(0,0),goal=(0,3))
        self.assertEqual(result['status'],'segment_footprint_blocked')
        self.assertEqual(result['waypoints_right_forward_m'],[])

    def test_bounds_and_goal_footprint_are_checked(self):
        result=validate_route(self.states,self.config,[[.125,.125],[.125,9.875]],start=(0,0),goal=(0,9.9))
        self.assertEqual(result['status'],'goal_footprint_blocked')
        self.states[self.config.cell(0,3)]=2
        result=validate_route(self.states,self.config,[[.125,.125],[.125,3.125]],start=(0,0),goal=(0,3))
        self.assertEqual(result['status'],'goal_footprint_blocked')

    def test_empty_malformed_and_nonfinite_routes_rejected(self):
        self.assertEqual(validate_route(self.states,self.config,[],start=(0,0),goal=(0,3))['status'],'no_planned_path')
        with self.assertRaises(ValueError):validate_route(self.states,self.config,[[float('nan'),0]],start=(0,0),goal=(0,3))
        with self.assertRaises(ValueError):validate_route(self.states,self.config,[[0,0]],start=(0,0),goal=(0,3),tracking_margin_m=-1)
