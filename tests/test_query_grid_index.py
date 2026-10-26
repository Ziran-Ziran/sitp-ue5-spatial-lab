from pathlib import Path
import math
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from local_depth_grid import GridConfig,clearance_mask
from query_grid_index import query_cell
from depth_grid_replay import plan_grid
from continuous_depth_path import validate_route


class QueryGridIndexTests(unittest.TestCase):
    def setUp(self):self.config=GridConfig()

    def test_generic_interior_lines_have_stable_index_with_both_error_signs(self):
        for axis in (0,1):
            for value in (-1,-.5,0,.5,1,2):
                point=[0,1];point[axis]=value;expected=self.config.cell(*point)
                for error in (-1e-13,0,1e-13):
                    perturbed=list(point);perturbed[axis]+=error
                    self.assertEqual(query_cell(self.config,perturbed,'boundary_snap_1e-9_cell'),expected)

    def test_legacy_and_beyond_band_keep_mathematical_floor(self):
        self.assertEqual(query_cell(self.config,[-4.218847493575595e-15,3],'legacy_floor'),(20,19))
        for value in (-1e-7,1e-7):
            self.assertEqual(query_cell(self.config,[value,3],'boundary_snap_1e-9_cell'),self.config.cell(value,3))

    def test_physical_bounds_never_pulled_inside(self):
        for point in ((-5-1e-12,0),(5,0),(0,-2-1e-12),(0,10)):
            self.assertEqual(query_cell(self.config,point,'boundary_snap_1e-9_cell'),self.config.cell(*point))

    def test_bad_policy_nonfinite_and_malformed_query_rejected(self):
        for point,policy in [([0,0],'invented'),([math.nan,0],'legacy_floor'),([0],'legacy_floor')]:
            with self.assertRaises(ValueError):query_cell(self.config,point,policy)

    def test_legacy_default_unchanged_and_explicit_indexing_audited(self):
        states=np.zeros(self.config.shape,dtype=np.int8);grid=dict(states=states,config=self.config,free=clearance_mask(states,.25,.34))
        goal=[-4.218847493575595e-15,3.0000000000002798];before=list(goal)
        old=plan_grid(grid,(0,0),goal);new=plan_grid(grid,(0,0),goal,query_cell_policy='boundary_snap_1e-9_cell')
        self.assertNotIn('query_indexing',old)
        self.assertEqual(new['query_indexing']['raw_cells'][1],[20,19]);self.assertEqual(new['query_indexing']['selected_cells'][1],[20,20])
        gate=validate_route(states,self.config,new['path_right_forward_m'],start=(0,0),goal=goal)
        self.assertEqual(gate['waypoints_right_forward_m'][-1],goal);self.assertEqual(goal,before)

    def test_numeric_indexing_does_not_override_unknown_endpoint_footprint(self):
        states=np.zeros(self.config.shape,dtype=np.int8);states[self.config.cell(-.125,3.125)]=-1
        before=states.copy();goal=[-1e-13,3]
        self.assertEqual(query_cell(self.config,goal,'boundary_snap_1e-9_cell'),(20,20))
        gate=validate_route(states,self.config,[[.125,.125],[.125,3.125]],start=(0,0),goal=goal)
        self.assertEqual(gate['status'],'goal_footprint_blocked');np.testing.assert_array_equal(states,before)
