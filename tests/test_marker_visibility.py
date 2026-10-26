from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from marker_visibility import depth_consistent_box_masks
from online_depth_policy import CALIBRATION


class MarkerVisibilityTests(unittest.TestCase):
    def setUp(self):
        self.entity=dict(entity_id='box',category='red_marker',position_cm=[300,0,100])
        v,u=np.indices((240,320));self.right=(u-159.5)/160;self.up=-(v-119.5)/160
        self.expected=(abs(self.right*2.9)<=.1)&(self.up*2.9+1.5>=.7)&(self.up*2.9+1.5<=1.3)
        self.depth=np.full((240,320),10000,np.uint16);self.depth[self.expected]=2900

    def masks(self,depth=None,**kwargs):
        return depth_consistent_box_masks(self.depth if depth is None else depth,CALIBRATION,
            camera_position_cm=kwargs.get('camera',[0,0,150]),heading_deg=kwargs.get('heading',0),
            entities=kwargs.get('entities',[self.entity]))

    def test_independent_front_face_projection_matches_exact_pixel_centers(self):
        report=self.masks();np.testing.assert_array_equal(report['entities'][0]['depth_consistent_mask'],self.expected)
        self.assertEqual(report['entities'][0]['depth_consistent_pixels'],int(self.expected.sum()))

    def test_nearer_occluder_invalid_depth_and_out_of_view_do_not_become_visible(self):
        for value in (1000,0,65535):
            depth=self.depth.copy();depth[self.expected]=value
            with self.subTest(value=value):self.assertEqual(self.masks(depth)['entities'][0]['depth_consistent_pixels'],0)
        self.assertEqual(self.masks(heading=180)['entities'][0]['projected_pixels'],0)
        self.assertEqual(self.masks(entities=[])['entities'],[])

    def test_translation_yaw_equivariance_and_no_camera_inside_box_label(self):
        entity=dict(self.entity,position_cm=[1000,2300,100])
        rotated=self.masks(camera=[1000,2000,150],heading=90,entities=[entity])
        np.testing.assert_array_equal(rotated['entities'][0]['depth_consistent_mask'],self.expected)
        self.assertEqual(self.masks(camera=[300,0,100])['entities'][0]['status'],'camera_inside_known_box')

    def test_depth_quantization_is_a_declared_bound_not_free_occlusion_space(self):
        for offset,visible in ((1,True),(2,True),(10,False),(-10,False)):
            depth=self.depth.copy();depth[self.expected]=2900+offset
            with self.subTest(offset=offset):self.assertEqual(bool(self.masks(depth)['entities'][0]['depth_consistent_pixels']),visible)

    def test_coincident_targets_are_ambiguous_and_not_double_labelled(self):
        duplicate=dict(self.entity,entity_id='second',category='blue_marker')
        report=self.masks(entities=[self.entity,duplicate])
        self.assertEqual(report['ambiguous_depth_consistent_pixels'],int(self.expected.sum()))
        self.assertTrue(all(row['depth_consistent_pixels']==0 for row in report['entities']))

    def test_bad_truth_pose_and_duplicate_ids_fail_closed(self):
        for entities in ([self.entity,self.entity],[dict(self.entity,position_cm=[0,0,float('nan')])]):
            with self.assertRaises(ValueError):self.masks(entities=entities)
        with self.assertRaises(ValueError):self.masks(heading=float('nan'))
