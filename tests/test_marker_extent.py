from pathlib import Path
import sys
import unittest
import math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from marker_extent import inspect_extent


class MarkerExtentTests(unittest.TestCase):
    def box(self,yaw):
        points=np.array([[r,f,u] for r in (-.1,.1) for f in (-.1,.1) for u in (-.3,.3)])
        angle=math.radians(yaw);rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        points[:,:2]=points[:,:2]@rotation.T
        return points+[2,5,1]

    def test_all_yaws_of_existing_known_box_are_within_upper_extent_bound(self):
        for yaw in range(-180,181,15):
            with self.subTest(yaw=yaw):self.assertTrue(inspect_extent(self.box(yaw))['accepted'])

    def test_oversized_same_color_wall_trim_and_tall_patch_are_rejected(self):
        for axis in range(3):
            points=self.box(0);points[-1,axis]+=1
            with self.subTest(axis=axis):
                result=inspect_extent(points);self.assertFalse(result['accepted'])
                self.assertIn(axis,result['violating_axes'])

    def test_partial_observation_has_no_minimum_size_or_completion(self):
        points=self.box(0)[:2]
        result=inspect_extent(points);self.assertTrue(result['accepted'])
        self.assertEqual(result['observed_points'],2)

    def test_quantization_bound_is_fixed_and_honors_touching_upper_limit(self):
        width=math.sqrt(.2**2+.2**2)+.002
        points=np.array([[0,0,0],[width,width,.602]])
        self.assertTrue(inspect_extent(points)['accepted'])
        points[-1,0]+=1e-8
        self.assertFalse(inspect_extent(points)['accepted'])

    def test_nonfinite_empty_and_unbounded_inputs_fail_closed(self):
        for points in (np.zeros((0,3)),np.array([[0,0,np.nan]]),np.zeros((76801,3)),np.zeros((4,2))):
            with self.subTest(shape=points.shape),self.assertRaises(ValueError):inspect_extent(points)

    def test_input_is_immutable_and_result_deterministic(self):
        points=self.box(30);before=points.copy()
        self.assertEqual(inspect_extent(points),inspect_extent(points));np.testing.assert_array_equal(points,before)
