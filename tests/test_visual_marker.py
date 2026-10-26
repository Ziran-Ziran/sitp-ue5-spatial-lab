from pathlib import Path
import sys
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from visual_marker import detect_markers
from online_depth_policy import CALIBRATION


class VisualMarkerTests(unittest.TestCase):
    def setUp(self):
        self.rgb=np.zeros((240,320,3),dtype=np.uint8);self.depth=np.zeros((240,320),dtype=np.uint16)

    def patch(self,rows,columns,color,depth):
        self.rgb[rows,columns]=color;self.depth[rows,columns]=depth

    def test_color_category_and_metric_position_come_from_pixels_only(self):
        self.patch(slice(140,146),slice(150,156),(0,0,255),2000)
        self.patch(slice(150,156),slice(200,206),(255,0,0),1500)
        report=detect_markers(self.rgb,self.depth,CALIBRATION)
        detections={d['category']:d for d in report['detections']}
        self.assertEqual(set(detections),{'red_marker','blue_marker'})
        red=detections['red_marker'];self.assertEqual(red['pixels'],36)
        self.assertAlmostEqual(red['position_right_forward_up_m'][0],-.0875)
        self.assertAlmostEqual(red['position_right_forward_up_m'][1],2)
        self.assertAlmostEqual(red['position_right_forward_up_m'][2],1.2125)
        self.assertEqual(red['depth_valid_fraction'],1)
        self.assertNotIn('world',str(report))

    def test_red_hue_wrap_and_dark_or_unsaturated_regions(self):
        hue=np.array([[[179,255,255]]],dtype=np.uint8);color=cv2.cvtColor(hue,cv2.COLOR_HSV2BGR)[0,0]
        self.patch(slice(140,146),slice(150,156),color,2000)
        self.assertEqual(detect_markers(self.rgb,self.depth,CALIBRATION)['detections'][0]['category'],'red_marker')
        for color in ((0,0,10),(128,128,128)):
            self.rgb[:]=color
            self.assertEqual(detect_markers(self.rgb,self.depth,CALIBRATION)['detections'],[])

    def test_missing_or_mixed_depth_is_reported_as_rejected_component(self):
        self.patch(slice(140,146),slice(150,156),(0,0,255),0)
        report=detect_markers(self.rgb,self.depth,CALIBRATION)
        self.assertEqual(report['detections'],[]);self.assertEqual(report['rejected'][0]['reason'],'insufficient_valid_depth')
        self.depth[140:143,150:156]=1000;self.depth[143:146,150:156]=3000
        report=detect_markers(self.rgb,self.depth,CALIBRATION)
        self.assertEqual(report['detections'],[]);self.assertEqual(report['rejected'][0]['reason'],'depth_discontinuity')

    def test_colored_floor_does_not_become_upright_marker(self):
        self.patch(slice(234,240),slice(150,156),(255,0,0),2000)
        report=detect_markers(self.rgb,self.depth,CALIBRATION)
        self.assertEqual(report['detections'],[]);self.assertEqual(report['rejected'][0]['reason'],'outside_marker_height_prior')

    def test_tiny_components_are_rejected_and_categories_are_kept_separate(self):
        self.patch(slice(140,142),slice(150,152),(0,0,255),2000)
        report=detect_markers(self.rgb,self.depth,CALIBRATION)
        self.assertEqual(report['detections'],[]);self.assertEqual(report['rejected'][0]['reason'],'too_few_pixels')

    def test_malformed_rgb_depth_or_intrinsics_are_rejected(self):
        for rgb,depth,calib in ((self.rgb.astype(float),self.depth,CALIBRATION),
                (self.rgb,self.depth.astype(float),CALIBRATION),(self.rgb,self.depth,dict(CALIBRATION,fx=0))):
            with self.assertRaises(ValueError):detect_markers(rgb,depth,calib)

    def test_many_color_components_have_an_explicit_bounded_budget(self):
        for row in range(140,190,4):
            for column in range(100,180,4):self.patch(slice(row,row+1),slice(column,column+1),(0,0,255),1500)
        report=detect_markers(self.rgb,self.depth,CALIBRATION)
        self.assertTrue(report['component_budget_exhausted']);self.assertEqual(report['components_processed'],64)
        self.assertEqual(len(report['rejected']),64)
