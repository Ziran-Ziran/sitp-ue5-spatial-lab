import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
from test_online_depth_policy import analytic_views
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from online_depth_policy import write_observation
from visual_marker_depth_policy import decide


class MarkerDepthPolicyTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name)/'input';self.root.mkdir()
        for name,depth in analytic_views().items():
            view=self.root/name;view.mkdir()
            cv2.imwrite(str(view/'rgb.png'),np.zeros((240,320,3),dtype=np.uint8));cv2.imwrite(str(view/'depth.png'),depth)
        self.request=dict(schema_version=1,attempt_id='12345678-1234-4234-8234-123456789abc',run_id='red-search',step_id=0,
            t_sim_s=0,input_dir=str(self.root),category='red_marker',consecutive_empty_views=0)

    def seal(self):
        path=self.root/'observation.json'
        if path.exists():path.unlink()
        write_observation(self.root,0)

    def marker(self,color,depth):
        rgb=cv2.imread(str(self.root/'level/rgb.png'),cv2.IMREAD_COLOR)
        values=cv2.imread(str(self.root/'level/depth.png'),cv2.IMREAD_UNCHANGED)
        rgb[140:146,156:162]=color;values[140:146,156:162]=depth
        cv2.imwrite(str(self.root/'level/rgb.png'),rgb);cv2.imwrite(str(self.root/'level/depth.png'),values);self.seal()

    def test_observed_color_selects_metric_standoff_and_safe_route_prefix(self):
        self.marker((0,0,255),2000);answer=decide(self.request)
        self.assertEqual(answer['action'],'move');self.assertEqual(answer['selected']['category'],'red_marker')
        self.assertAlmostEqual(answer['geometry_goal_right_forward_m'][1],1.1,places=3)
        route=answer['waypoints_right_forward_m'];self.assertEqual(route[0],[0,0]);self.assertLessEqual(route[-1][1],1)

    def test_wrong_color_searches_four_headings_and_retains_absence(self):
        self.marker((255,0,0),2000)
        for count in range(3):
            answer=decide(dict(self.request,consecutive_empty_views=count))
            self.assertEqual(answer['action'],'rotate');self.assertEqual(answer['relative_yaw_delta_deg'],90)
        answer=decide(dict(self.request,consecutive_empty_views=3))
        self.assertEqual(answer['action'],'stop');self.assertEqual(answer['reason'],'target_not_observed_after_scan')

    def test_reached_candidate_requires_fresh_classified_metric_pixels(self):
        self.marker((0,0,255),1000)
        answer=decide(self.request);self.assertEqual(answer['action'],'candidate_reached')
        self.assertEqual(answer['waypoints_right_forward_m'],[])

    def test_privileged_goal_catalog_pose_or_category_alias_is_rejected(self):
        self.seal()
        for key in ('goal_cm','goal_right_forward_m','entities','pose','map'):
            request=copy.deepcopy(self.request);request[key]={}
            with self.subTest(key=key),self.assertRaises(ValueError):decide(request)
        with self.assertRaises(ValueError):decide(dict(self.request,category='chair'))

    def test_schema_two_requires_explicit_variant_and_preserves_schema_one(self):
        self.marker((0,0,255),2000)
        legacy=decide(self.request)
        self.assertNotIn('planner_variant',legacy)
        for variant in ('legacy_centers','continuous_square','continuous_square_shortcuts'):
            answer=decide(dict(self.request,schema_version=2,planner_variant=variant))
            self.assertEqual(answer['schema_version'],2);self.assertEqual(answer['planner_variant'],variant)
            self.assertEqual(answer['geometry']['planner_variant'],variant)
        with self.assertRaises(ValueError):decide(dict(self.request,schema_version=2))
        with self.assertRaises(ValueError):decide(dict(self.request,planner_variant='legacy_centers'))
