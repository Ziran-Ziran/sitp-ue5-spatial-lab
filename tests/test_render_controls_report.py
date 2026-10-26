import importlib.util
from pathlib import Path
import unittest
import numpy as np


class RenderControlReportTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).resolve().parents[1]/'ue5/tools/render_controls_report.py'
        spec=importlib.util.spec_from_file_location('render_controls_report',path)
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        color=np.zeros((240,320,3),np.uint8);color[:,:,0]=255
        color[22:87,46:111]=[0,0,255]
        color[152:217,209:274]=[0,255,0]
        depth=np.full((240,320),10000,np.uint16)
        depth[22:87,46:111]=4900;depth[152:217,209:274]=4900
        self.frames={
            'pose-a':(color,depth),'pose-b':(color.copy(),np.full_like(depth,8000)),
            'pose-a-repeat':(color.copy(),depth.copy()),
            'exposure-dark':(np.zeros_like(color),depth.copy()),
            'empty-backward':(np.zeros_like(color),np.zeros_like(depth))}

    def test_axis_depth_color_orientation_and_repeat_controls_pass(self):
        self.assertTrue(self.module.validate_controls(self.frames)['passed'])

    def test_radial_depth_is_not_mislabeled_optical_axis(self):
        color,depth=self.frames['pose-a'];depth[230,10]=15000
        self.assertFalse(self.module.validate_controls(self.frames)['passed'])

    def test_stale_depth_after_camera_translation_is_rejected(self):
        self.frames['pose-b']=(self.frames['pose-b'][0],self.frames['pose-a'][1].copy())
        self.assertFalse(self.module.validate_controls(self.frames)['passed'])

    def test_vertical_flip_or_channel_swap_is_rejected(self):
        color,depth=self.frames['pose-a']
        self.frames['pose-a']=(color[::-1].copy(),depth[::-1].copy())
        self.assertFalse(self.module.validate_controls(self.frames)['passed'])

    def test_depth_format_and_shape_are_required(self):
        color,depth=self.frames['pose-a'];self.frames['pose-a']=(color,depth.astype(np.uint8))
        with self.assertRaises(ValueError):self.module.validate_controls(self.frames)

    def test_exposure_parameter_must_change_color_but_not_depth(self):
        self.frames['exposure-dark']=(self.frames['pose-a'][0].copy(),self.frames['pose-a'][1].copy())
        self.assertFalse(self.module.validate_controls(self.frames)['passed'])
