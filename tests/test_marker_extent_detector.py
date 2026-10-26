from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from visual_marker import detect_markers
from online_depth_policy import CALIBRATION


class MarkerExtentDetectorTests(unittest.TestCase):
    def setUp(self):
        self.color=np.zeros((240,320,3),np.uint8);self.depth=np.zeros((240,320),np.uint16)

    def patch(self,rows,columns,color=(255,0,0)):
        self.color[rows,columns]=color;self.depth[rows,columns]=2000

    def detect(self,variant):return detect_markers(self.color,self.depth,CALIBRATION,perception_variant=variant)

    def test_legacy_explicit_variant_preserves_exact_default_report(self):
        self.patch(slice(140,146),slice(150,156))
        self.assertEqual(self.detect('hsv_depth_v1'),detect_markers(self.color,self.depth,CALIBRATION))

    def test_small_observed_patch_retains_same_pixel_only_position_with_extent_record(self):
        self.patch(slice(140,146),slice(150,156))
        old=self.detect('hsv_depth_v1');new=self.detect('known_box_extent_v1')
        self.assertEqual(len(new['detections']),1)
        self.assertEqual(new['detections'][0]['position_right_forward_up_m'],old['detections'][0]['position_right_forward_up_m'])
        self.assertTrue(new['detections'][0]['extent_prior']['accepted'])

    def test_wide_trim_and_tall_panel_are_accepted_by_color_but_rejected_by_known_extent(self):
        for rows,columns in ((slice(140,146),slice(80,180)),(slice(130,190),slice(150,156))):
            self.color[:]=0;self.depth[:]=0;self.patch(rows,columns)
            self.assertEqual(len(self.detect('hsv_depth_v1')['detections']),1)
            report=self.detect('known_box_extent_v1');self.assertEqual(report['detections'],[])
            self.assertEqual(report['rejected'][0]['reason'],'outside_known_marker_box_extent')

    def test_nearer_oversized_color_does_not_hide_farther_valid_component(self):
        self.patch(slice(140,146),slice(40,100));self.depth[140:146,40:100]=1500
        self.patch(slice(140,146),slice(200,206))
        old=self.detect('hsv_depth_v1');new=self.detect('known_box_extent_v1')
        self.assertEqual(len(old['detections']),2);self.assertEqual(len(new['detections']),1)
        self.assertEqual(new['detections'][0]['bbox_xywh'],[200,140,6,6])

    def test_unknown_variant_fails_closed(self):
        with self.assertRaises(ValueError):self.detect('engine_box_truth')
