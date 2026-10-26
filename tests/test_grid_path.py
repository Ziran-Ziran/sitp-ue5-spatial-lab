from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from grid_path import astar


class GridPathTests(unittest.TestCase):
    def test_detour_returns_collision_free_metric_path(self):
        free=np.ones((7,7),dtype=bool);free[1:6,3]=False
        result=astar(free,(3,1),(3,5),resolution_m=.25)
        self.assertEqual(result['status'],'path_found')
        self.assertEqual(result['cells'][0],[3,1]);self.assertEqual(result['cells'][-1],[3,5])
        self.assertTrue(all(free[tuple(cell)] for cell in result['cells']))
        self.assertGreater(result['path_length_m'],1)

    def test_diagonal_cannot_cut_blocked_corners(self):
        result=astar(np.array([[True,False],[False,True]]),(0,0),(1,1))
        self.assertEqual(result['status'],'no_path')

    def test_barrier_and_invalid_endpoints_have_explicit_failures(self):
        free=np.ones((5,5),dtype=bool);free[2,:]=False
        self.assertEqual(astar(free,(0,0),(4,4))['status'],'no_path')
        self.assertEqual(astar(free,(2,0),(4,4))['status'],'start_blocked')
        self.assertEqual(astar(free,(0,0),(2,0))['status'],'goal_blocked')
        self.assertEqual(astar(free,(-1,0),(4,4))['status'],'start_outside')
        self.assertEqual(astar(free,(0,0),(5,4))['status'],'goal_outside')

    def test_same_cell_and_bounded_expansions(self):
        free=np.ones((8,8),dtype=bool)
        self.assertEqual(astar(free,(1,1),(1,1))['path_length_m'],0)
        self.assertEqual(astar(free,(0,0),(7,7),max_expansions=1)['status'],'expansion_budget')
        with self.assertRaises(ValueError):astar(free,(0,0),(7,7),resolution_m=float('nan'))

    def test_open_diagonal_has_optimal_metric_length_and_is_deterministic(self):
        free=np.ones((5,5),dtype=bool)
        first=astar(free,(0,0),(4,4),resolution_m=.25)
        self.assertAlmostEqual(first['path_length_m'],2**.5)
        self.assertEqual(first['cells'],astar(free,(0,0),(4,4))['cells'])
