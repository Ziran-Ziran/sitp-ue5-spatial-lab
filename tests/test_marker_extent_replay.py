import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import test_visual_marker_depth_policy as fixtures
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from visual_marker_depth_policy import decide
from marker_extent_replay import evaluate_record


class MarkerExtentReplayTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.MarkerDepthPolicyTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.marker((0,0,255),2000);self.fixture=fixture
        self.path=fixture.root.parent/'execution.json'
        self.record=dict(evidence_kind='ue_game_world_rgb_marker_depth_target_search',
            runs=[dict(run_id=fixture.request['run_id'],input=dict(category='red_marker'),
                execution=dict(actions=[dict(status='complete',step_id=0,request=fixture.request,response=decide(fixture.request))]))])
        self.path.write_text(json.dumps(self.record),encoding='utf-8')

    def test_pixel_only_replay_preserves_legacy_and_keeps_both_selected_reports(self):
        original=Path.open
        def guarded(path,*args,**kwargs):
            if path.name=='truth.json':raise AssertionError('truth read')
            return original(path,*args,**kwargs)
        with patch.object(Path,'open',guarded):report=evaluate_record(self.path)
        self.assertEqual(report['summary']['observations'],1)
        self.assertEqual(report['summary']['hsv_depth_v1']['selected_observations'],1)
        self.assertEqual(report['summary']['known_box_extent_v1']['selected_observations'],1)
        self.assertNotIn('world',report['observations'][0])

    def test_changed_saved_baseline_detector_cannot_be_compared_as_original(self):
        self.record['runs'][0]['execution']['actions'][0]['response']['detection_report']['detections']=[]
        self.path.write_text(json.dumps(self.record),encoding='utf-8')
        with self.assertRaisesRegex(ValueError,'baseline detector'):evaluate_record(self.path)
