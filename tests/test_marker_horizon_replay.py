import copy
import json
from pathlib import Path
import sys
import unittest
import test_visual_marker_depth_policy as fixtures
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from visual_marker_depth_policy import decide
from marker_horizon_replay import evaluate_record


class MarkerHorizonReplayTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.MarkerDepthPolicyTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.marker((0,0,255),2000);self.path=fixture.root.parent/'execution.json'
        request=dict(fixture.request,schema_version=2,planner_variant='legacy_centers')
        self.raw=dict(runs=[dict(run_id=request['run_id'],input=dict(category='red_marker'),
            execution=dict(actions=[dict(status='complete',step_id=0,request=request,response=decide(request))]))])
        self.path.write_text(json.dumps(self.raw),encoding='utf-8')

    def test_both_horizons_use_same_saved_pixels_and_preserve_old_three_meter_response(self):
        result=evaluate_record(self.path)
        self.assertEqual(result['summary']['observations'],1)
        self.assertEqual(result['summary']['fixed_three_meters']['actions'],dict(move=1))
        self.assertEqual(result['summary']['execution_prefix_one_meter']['actions'],dict(move=1))

    def test_modified_historical_action_cannot_be_counted_as_original(self):
        self.raw['runs'][0]['execution']['actions'][0]['response']['action']='candidate_reached'
        self.path.write_text(json.dumps(self.raw),encoding='utf-8')
        with self.assertRaises(ValueError):evaluate_record(self.path)
