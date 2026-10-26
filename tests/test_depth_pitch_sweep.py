"""Analytic calibrated views, not sampled engine truth."""
from pathlib import Path
import sys
import unittest
import numpy as np
import test_local_depth_grid as geometry

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from local_depth_grid import GridConfig,unproject,depth_grid,multiview_grid
from grid_path import astar


def floor_view(pitch_down_deg):
    theta=np.deg2rad(pitch_down_deg)
    ray_down=(np.arange(240)[:,None]-119.5)/160
    denominator=np.sin(theta)+ray_down*np.cos(theta)
    z=np.divide(1500,denominator,out=np.zeros((240,1)),where=denominator>1e-8)
    z[(z<=0)|(z>30000)]=0
    return np.repeat(np.rint(z).astype(np.uint16),320,axis=1)


class DepthPitchSweepTests(unittest.TestCase):
    def test_nadir_axes_and_zero_reference_floor(self):
        depth=np.zeros((240,320),np.uint16);depth[199,239]=1500
        point=unproject(depth,geometry.CALIBRATION,eye_height_m=1.5,pitch_down_deg=90)
        np.testing.assert_allclose(point,[[.7453125,-.7453125,0]],atol=1e-12)

    def test_all_views_reconstruct_same_horizontal_plane(self):
        for pitch in (0,45,90):
            points=unproject(floor_view(pitch),geometry.CALIBRATION,eye_height_m=1.5,pitch_down_deg=pitch)
            self.assertLess(abs(points[:,2]).max(),.001)
        with self.assertRaises(ValueError):unproject(floor_view(0),geometry.CALIBRATION,eye_height_m=1.5,pitch_down_deg=float('nan'))

    def test_new_views_reveal_actual_start_without_inventing_floor(self):
        config=GridConfig()
        level=depth_grid(floor_view(0),geometry.CALIBRATION,config)
        combined=multiview_grid([(pitch,floor_view(pitch)) for pitch in (0,45,90)],geometry.CALIBRATION,config)
        start,goal=config.cell(0,0),config.cell(0,4)
        self.assertFalse(level['free'][start]);self.assertTrue(combined['free'][start])
        self.assertEqual(astar(combined['free'],start,goal)['status'],'path_found')

    def test_hazards_across_views_override_floor_support(self):
        bad=geometry.floor_depth();bad[239,160]=3000
        combined=multiview_grid([(0,bad),(45,floor_view(45)),(90,floor_view(90))],geometry.CALIBRATION,GridConfig())
        cell=combined['config'].cell(0,3)
        self.assertEqual(combined['states'][cell],2)
        self.assertFalse(combined['free'][cell])

    def test_missing_views_are_not_silently_replaced(self):
        with self.assertRaises(ValueError):multiview_grid([],geometry.CALIBRATION,GridConfig())
