import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from test_online_depth_policy import analytic_views
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from online_depth_policy import CALIBRATION,write_observation
from visual_marker import detect_markers
from marker_visibility_report import analyze_record,control_acceptance


class MarkerVisibilityReportTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup);self.root=Path(temporary.name)
        self.pixels=self.root/'input';self.pixels.mkdir()
        v,u=np.indices((240,320));r=(u-159.5)/160;up=-(v-119.5)/160
        mask=(abs(r*2.9)<=.1)&(up*2.9+1.5>=.7)&(up*2.9+1.5<=1.3)
        for name,depth in analytic_views().items():
            (self.pixels/name).mkdir();color=np.zeros((240,320,3),np.uint8)
            if name=='level':depth[mask]=2900;color[mask]=(0,0,255)
            cv2.imwrite(str(self.pixels/name/'rgb.png'),color);cv2.imwrite(str(self.pixels/name/'depth.png'),depth)
        write_observation(self.pixels,0)
        rgb=cv2.imread(str(self.pixels/'level/rgb.png'));depth=cv2.imread(str(self.pixels/'level/depth.png'),cv2.IMREAD_UNCHANGED)
        self.run=dict(run_id='r',category='red_marker',environment_case='all_markers')
        self.event=dict(step_id=0,status='complete',heading_deg=0,feet_cm=[0,0,0],
            request=dict(run_id='r',category='red_marker',input_dir=str(self.pixels),t_sim_s=0),
            response=dict(detection_report=detect_markers(rgb,depth,CALIBRATION)),
            captures=[dict(view='level',camera_position_cm=[0,0,150],camera_rotation_deg=[0,0,0])])
        self.record=dict(runs=[dict(run_id='r',input=self.run,execution=dict(actions=[self.event]))])
        self.tasks=dict(runs=[self.run]);self.truth=dict(entities=[dict(entity_id='target',category='red_marker',position_cm=[300,0,100])])

    def analyze(self):return analyze_record(self.tasks,self.record,self.truth)

    def test_depth_proxy_color_and_accepted_detection_have_separate_counts(self):
        report=self.analyze();self.assertEqual(report['summary']['complete_observations'],1)
        self.assertEqual(report['summary']['observations_with_requested_proxy_target'],1)
        self.assertGreater(report['summary']['requested_proxy_pixels'],9)
        self.assertEqual(report['summary']['requested_proxy_pixels'],report['summary']['requested_proxy_hsv_pixels'])
        self.assertEqual(report['summary']['hsv_depth_v1']['true_point_associations'],1)

    def test_hidden_truth_is_excluded_without_informing_detector_and_incomplete_is_kept(self):
        self.run['environment_case']='red_hidden'
        report=self.analyze();self.assertEqual(report['summary']['observations_with_requested_proxy_target'],0)
        self.assertEqual(report['summary']['hsv_depth_v1']['selected_observations'],1)
        self.event['status']='capture_failed';report=self.analyze()
        self.assertEqual(report['summary']['observations'],1);self.assertEqual(report['summary']['complete_observations'],0)

    def test_detectors_receive_only_pixels_intrinsics_and_explicit_variant(self):
        original=detect_markers;seen=[]
        def guarded(color,depth,calibration,**kwargs):
            seen.append(set(kwargs));return original(color,depth,calibration,**kwargs)
        with patch('marker_visibility_report.detect_markers',side_effect=guarded):self.analyze()
        self.assertEqual(seen,[{'perception_variant'},{'perception_variant'}])

    def test_nonlevel_camera_cannot_be_scored_with_a_yaw_only_proxy(self):
        self.event['captures'][0]['camera_rotation_deg']=[45,0,0]
        with self.assertRaises(ValueError):self.analyze()

    def test_visibility_or_truth_module_never_enters_visual_worker_import_closure(self):
        from marker_extent_replay import source_snapshot
        sources=source_snapshot('visual_marker_depth_policy')
        self.assertNotIn('marker_visibility.py',sources);self.assertNotIn('marker_visibility_report.py',sources)
        self.assertNotIn('visual_marker_report.py',sources)

    def test_missing_projection_or_wrong_hidden_support_fails_control_gate(self):
        entity=dict(requested=True,depth_consistent_pixels=9,models={'hsv_depth_v1':dict(accepted_supported_pixels=9)})
        names=[f'start_{index:02d}-{category}' for index in range(4) for category in ('red_marker','blue_marker')]+['immediate']
        rows=[dict(run_id=name,entities=[copy.deepcopy(entity)]) for name in names]
        rows.extend(dict(run_id=name,entities=[]) for name in ('missing','occluded'))
        report=dict(observations=rows,summary=dict(task_runs=12,saved_runs=12,observations=11,complete_observations=11),
            original_navigation_overall=dict(runs=12,captures=11))
        self.assertTrue(control_acceptance(report)['passed'])
        rows[-1]['entities']=[entity];self.assertFalse(control_acceptance(report)['passed'])

    def test_visible_colored_target_omitted_by_component_budget_is_not_a_color_miss(self):
        color=cv2.imread(str(self.pixels/'level/rgb.png'));depth=cv2.imread(str(self.pixels/'level/depth.png'),cv2.IMREAD_UNCHANGED)
        for top in (0,22,44,190,212):
            for left in range(0,308,22):
                color[top:top+20,left:left+20]=(0,0,255);depth[top:top+20,left:left+20]=1000
        cv2.imwrite(str(self.pixels/'level/rgb.png'),color);cv2.imwrite(str(self.pixels/'level/depth.png'),depth)
        (self.pixels/'observation.json').unlink();write_observation(self.pixels,0)
        self.event['response']['detection_report']=detect_markers(color,depth,CALIBRATION)
        summary=self.analyze()['summary']
        self.assertEqual(summary['requested_proxy_eligible_pairs_with_color_pixels'],1)
        self.assertEqual(summary['hsv_depth_v1']['proxy_eligible_pairs_with_accepted_pixels'],0)
        self.assertEqual(summary['hsv_depth_v1']['proxy_eligible_pairs_with_unprocessed_color_components'],1)
