from pathlib import Path
import sys
import unittest
import math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from marker_goal_horizon import observed_goal,POLICIES
from local_depth_grid import GridConfig,points_grid
from continuous_grid_planner import plan_route


class MarkerGoalHorizonTests(unittest.TestCase):
    def test_fixed_goal_matches_original_arithmetic_and_short_goal_uses_execution_prefix(self):
        for r,f in ((0,4),(-3,4),(2,-2),(.01,1.4)):
            distance=float(np.linalg.norm([r,f]));travel=min(3,distance-.9)
            self.assertEqual(observed_goal([r,f,1],distance,POLICIES[0]),[r*travel/distance,f*travel/distance])
            goal=observed_goal([r,f,1],distance,POLICIES[1])
            self.assertAlmostEqual(math.hypot(*goal),min(1,distance-.9))

    def test_invalid_range_variant_and_nonfinite_point_fail_closed(self):
        for point,distance,policy in (([0,4,1],1,POLICIES[0]),([0,1,1],1,POLICIES[1]),
                ([0,float('nan'),1],4,POLICIES[1]),([0,4,1],4,'truth_tuned')):
            with self.assertRaises(ValueError):observed_goal(point,distance,policy)

    def test_short_goal_can_fail_at_wall_even_when_far_goal_has_a_detour(self):
        config=GridConfig();r,f=np.meshgrid(np.arange(-4.9,5,.075),np.arange(-1.9,10,.075))
        floor=np.column_stack((r.ravel(),f.ravel(),np.zeros(r.size)))
        grid=points_grid(np.concatenate((floor,np.array([[0,1,.7]]))),config,condition='analytic_wall_horizon_control')
        far=observed_goal([0,4,1],4,POLICIES[0]);near=observed_goal([0,4,1],4,POLICIES[1])
        self.assertEqual(plan_route(grid,(0,0),far,'continuous_square_shortcuts')['continuous_gate']['status'],'approved')
        self.assertNotEqual(plan_route(grid,(0,0),near,'continuous_square_shortcuts')['continuous_gate']['status'],'approved')
