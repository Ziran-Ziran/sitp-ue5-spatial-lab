from pathlib import Path
import sys
import unittest
import math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from local_depth_grid import GridConfig,points_grid
from support_plane_grid import fit_support_plane,support_plane_points_grid,support_plane_grid
from continuous_grid_planner import plan_route


def floor_points(a=0,b=0,c=0):
    r,f=np.meshgrid(np.arange(-4.9,4.95,.075),np.arange(-1.9,9.95,.075))
    return np.column_stack((r.ravel(),f.ravel(),(a*r+b*f+c).ravel()))


class SupportPlaneGridTests(unittest.TestCase):
    def setUp(self):self.config=GridConfig()

    def test_flat_uphill_downhill_planes_keep_observed_support(self):
        for angle in (0,15,-15,30,-30):
            points=floor_points(b=math.tan(math.radians(angle)));grid=support_plane_points_grid(points,self.config)
            self.assertEqual(grid['support_plane']['status'],'supported')
            self.assertAlmostEqual(grid['support_plane']['coefficients_right_forward_up'][1],math.tan(math.radians(angle)),places=8)
            self.assertEqual(plan_route(grid,(0,0),(0,3),'continuous_square_shortcuts')['continuous_gate']['status'],'approved')
            if abs(angle)>=15:
                old=points_grid(points,self.config,condition='horizontal_prior')
                self.assertNotEqual(plan_route(old,(0,0),(0,3),'legacy_centers')['continuous_gate']['status'],'approved')

    def test_steep_and_offset_planes_are_unsupported(self):
        for points in (floor_points(b=math.tan(math.radians(50))),floor_points(c=.3)):
            grid=support_plane_points_grid(points,self.config)
            self.assertNotEqual(grid['support_plane']['status'],'supported');self.assertTrue(np.all(grid['states']==-1))
            self.assertEqual(grid['in_bounds_points'],len(points));self.assertEqual(grid['unclassified_samples'],len(points))

    def test_unknown_hole_is_not_filled_by_fitted_plane(self):
        points=floor_points(b=.2);keep=~((abs(points[:,0])<.6)&(points[:,1]>2)&(points[:,1]<4))
        grid=support_plane_points_grid(points[keep],self.config)
        self.assertEqual(grid['support_plane']['status'],'supported');self.assertEqual(grid['states'][self.config.cell(0,3)],-1)
        self.assertEqual(plan_route(grid,(0,0),(0,3),'continuous_square')['planning']['status'],'goal_unobserved')

    def test_missing_surrounding_support_cannot_extrapolate_under_agent(self):
        points=floor_points();grid=support_plane_points_grid(points[points[:,1]>.2],self.config)
        self.assertNotEqual(grid['support_plane']['status'],'supported');self.assertFalse(grid['free'].any())

    def test_wall_and_drop_relative_to_plane_remain_blocked(self):
        points=floor_points(b=.2)
        wall=np.array([[r,1.5,.3+h] for r in np.arange(-4.9,4.95,.075) for h in np.arange(.15,1.7,.1)])
        grid=support_plane_points_grid(np.concatenate((points,wall)),self.config)
        self.assertEqual(grid['states'][self.config.cell(0,1.5)],1)
        self.assertNotEqual(plan_route(grid,(0,0),(0,3),'continuous_square')['continuous_gate']['status'],'approved')
        points[points[:,1]>2,2]-=1
        grid=support_plane_points_grid(points,self.config)
        self.assertEqual(grid['states'][self.config.cell(0,3)],2)

    def test_outliers_are_not_erased_and_inputs_are_immutable(self):
        points=floor_points(b=.2);outliers=points[::7].copy();outliers[:,2]+=.4;points=np.concatenate((points,outliers));before=points.copy()
        first=support_plane_points_grid(points,self.config);second=support_plane_points_grid(points,self.config)
        self.assertEqual(first['support_plane'],second['support_plane']);self.assertEqual(first['support_plane']['status'],'supported')
        self.assertGreater(np.count_nonzero(first['states']==1),0);np.testing.assert_array_equal(points,before)

    def test_empty_nonfinite_and_unbounded_points(self):
        self.assertNotEqual(fit_support_plane(np.empty((0,3)),self.config)['status'],'supported')
        for points in (np.array([[0,0,np.nan]]),np.zeros((250001,3)),np.zeros((4,2))):
            with self.assertRaises(ValueError):fit_support_plane(points,self.config)

    def test_abrupt_steps_inside_seed_region_do_not_become_safe_crossings(self):
        for height in (-1,-.4,-.2,.2,.4,1):
            for edge in (0,.25,.5):
                with self.subTest(height=height,edge=edge):
                    points=floor_points();points[points[:,1]>edge,2]+=height
                    grid=support_plane_points_grid(points,self.config)
                    self.assertNotEqual(plan_route(grid,(0,0),(0,1),'continuous_square_shortcuts')['continuous_gate']['status'],'approved')

    def test_fitted_floor_offset_cannot_hide_ceiling_below_actual_capsule_top(self):
        ground=floor_points(c=-.02);ceiling=ground.copy();ceiling[:,2]=1.75
        grid=support_plane_points_grid(np.concatenate((ground,ceiling)),self.config)
        self.assertNotEqual(plan_route(grid,(0,0),(0,3),'continuous_square_shortcuts')['continuous_gate']['status'],'approved')

    def test_sloped_parallel_ceiling_cannot_ignore_cross_footprint_capsule(self):
        for angle in (-30,30):
            for axis in ('right','forward'):
                with self.subTest(angle=angle,axis=axis):
                    slope=math.tan(math.radians(angle))
                    ground=floor_points(a=slope if axis=='right' else 0,b=slope if axis=='forward' else 0)
                    ceiling=ground.copy();ceiling[:,2]+=1.77
                    # A point .17m downhill lies below the upright capsule dome.
                    ceiling_up=1.77-abs(slope)*.17
                    capsule_up=1.76-.34+math.sqrt(.34**2-.17**2)
                    self.assertLess(ceiling_up,capsule_up)
                    grid=support_plane_points_grid(np.concatenate((ground,ceiling)),self.config)
                    self.assertEqual(grid['support_plane']['status'],'supported')
                    self.assertNotEqual(plan_route(grid,(0,0),(0,3),'continuous_square_shortcuts')['continuous_gate']['status'],'approved')

    def test_independent_three_view_optical_depth_plane(self):
        calibration=dict(w=320,h=240,fx=160,fy=160,cx=159.5,cy=119.5);v,u=np.indices((240,320));a=.1;b=.2;views=[]
        for pitch in (0,45,90):
            theta=math.radians(pitch);r=(u-159.5)/160;local_up=-(v-119.5)/160
            f=np.cos(theta)+local_up*np.sin(theta);up=-np.sin(theta)+local_up*np.cos(theta)
            denominator=up-a*r-b*f
            with np.errstate(divide='ignore',invalid='ignore'):depth=-1.5/denominator
            depth=np.where((depth>0)&(depth<=30),np.round(depth*1000),0).astype(np.uint16);views.append((pitch,depth))
        grid=support_plane_grid(views,calibration,self.config)
        self.assertEqual(grid['support_plane']['status'],'supported');np.testing.assert_allclose(grid['support_plane']['coefficients_right_forward_up'],[a,b,0],atol=.001)
        zeros=[(pitch,np.zeros((240,320),dtype=np.uint16)) for pitch in (0,45,90)]
        self.assertFalse(support_plane_grid(zeros,calibration,self.config)['free'].any())
