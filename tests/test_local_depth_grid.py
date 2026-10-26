"""Analytic optical-depth geometry, independent of engine assets or truth."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from local_depth_grid import GridConfig,depth_grid,unproject,clearance_mask

CALIBRATION=dict(w=320,h=240,fx=160,fy=160,cx=159.5,cy=119.5)


def floor_depth():
    row=np.arange(240)[:,None]-119.5
    distance=np.divide(1500*160,row,out=np.zeros((240,1)),where=row>0)
    distance[(distance>30000)|(distance<=0)]=0
    return np.repeat(np.rint(distance).astype(np.uint16),320,axis=1)


class LocalDepthGridTests(unittest.TestCase):
    def test_asymmetric_optical_axes_and_metric_units(self):
        depth=np.zeros((240,320),np.uint16);depth[199,239]=2000
        points=unproject(depth,CALIBRATION,eye_height_m=1.5)
        np.testing.assert_allclose(points,[[.99375,2,.50625]])

    def test_level_floor_supported_but_unobserved_near_camera_stays_unknown(self):
        grid=depth_grid(floor_depth(),CALIBRATION,GridConfig())
        self.assertEqual(grid['states'][grid['config'].cell(0,3)],0)
        self.assertEqual(grid['states'][grid['config'].cell(0,0)],-1)
        self.assertGreater(np.count_nonzero(grid['states']==0),100)
        self.assertFalse(np.any(grid['states']==1))

    def test_wall_overrides_floor_and_does_not_fill_hidden_ground(self):
        depth=floor_depth();v,u=np.indices(depth.shape)
        height=1.5-(v-119.5)/160*3
        hit=(abs((u-159.5)/160*3)<.5)&(height>=0)&(height<=2)
        depth[hit & ((depth==0)|(depth>3000))]=3000
        result=depth_grid(depth,CALIBRATION,GridConfig())
        config=result['config'];states=result['states']
        self.assertEqual(states[config.cell(0,3)],1)
        self.assertEqual(states[config.cell(0,4)],-1)

    def test_missing_and_saturated_depth_never_create_free_space(self):
        depth=np.zeros((240,320),np.uint16);depth[:,160:]=65535
        result=depth_grid(depth,CALIBRATION,GridConfig())
        self.assertTrue(np.all(result['states']==-1))
        self.assertFalse(clearance_mask(result['states'],.25,.34).any())

    def test_unknown_and_bounds_receive_body_clearance(self):
        states=np.zeros((9,9),np.int8);states[4,4]=-1
        free=clearance_mask(states,.25,.34)
        self.assertFalse(free[4,4]);self.assertFalse(free[4,5])
        self.assertFalse(free[0,0]);self.assertTrue(free[2,2])

    def test_below_reference_floor_is_hazard_and_overhead_does_not_block(self):
        # Same depth viewed from an incorrect higher reference has no floor
        # support. This is an explicit limitation of the height prior.
        result=depth_grid(floor_depth(),CALIBRATION,GridConfig(eye_height_m=1.1))
        self.assertFalse(np.any(result['states']==0))
        self.assertTrue(np.any(result['states']==2))
        depth=np.zeros((240,320),np.uint16);depth[0,160]=3000
        result=depth_grid(depth,CALIBRATION,GridConfig())
        self.assertTrue(np.all(result['states']==-1))

    def test_observed_drop_overrides_mixed_floor_support_in_same_cell(self):
        depth=floor_depth();depth[239,160]=3000
        result=depth_grid(depth,CALIBRATION,GridConfig())
        cell=result['config'].cell(0,3)
        self.assertEqual(result['states'][cell],2)
        self.assertFalse(result['free'][cell])

    def test_body_radius_blocks_narrow_unknown_bounded_corridor(self):
        narrow=np.full((15,15),-1,dtype=np.int8);narrow[:,6:9]=0
        wide=np.full((15,15),-1,dtype=np.int8);wide[:,5:10]=0
        self.assertFalse(clearance_mask(narrow,.25,.34)[7,7])
        self.assertTrue(clearance_mask(wide,.25,.34)[7,7])

    def test_malformed_depth_intrinsics_or_config_is_rejected(self):
        with self.assertRaises(ValueError):unproject(np.zeros((240,320)),CALIBRATION,eye_height_m=1.5)
        with self.assertRaises(ValueError):unproject(floor_depth(),dict(CALIBRATION,fx=0),eye_height_m=1.5)
        with self.assertRaises(ValueError):GridConfig(resolution_m=0)
        with self.assertRaises(ValueError):GridConfig(eye_height_m=float('nan'))
        with self.assertRaises(ValueError):GridConfig(right_min_m=-1,right_max_m=1.1)
