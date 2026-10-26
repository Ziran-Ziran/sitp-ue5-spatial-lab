import json
from pathlib import Path
import sys
import unittest
import test_visual_marker_report as support
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from online_depth_tasks import build_tasks as source_tasks
from city_marker_tasks import build_tasks
from visual_marker_depth_policy import decide
from visual_marker_report import evaluate


class CityMarkerReportTests(unittest.TestCase):
    def test_city_exposure_full_denominator_and_independent_candidate_truth(self):
        fixture=support.VisualMarkerReportTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        frames=[dict(sequence='sequence_000',source_run_id='source',frame_id=0,source_frame_id=0,t_source_sim_s=0,start_cm=[230,-200,0],heading_deg=0)]
        original=source_tasks('/Game/SpatialLab/Base','a'*64,frames,exposure=13,source_binding={})
        tasks=build_tasks('/Game/SpatialLab/Markers','d'*64,original,dict(base_map_sha256='a'*64,fixture_plan_sha256='b'*64,fixture_report_sha256='c'*64),'legacy_centers')
        task=tasks['runs'][0];old=fixture.root/'run_008';new=fixture.root/'run_000';old.rename(new)
        event=fixture.raw['actions'][0];folder=new/'step_000';event['folder']=str(folder)
        event['request'].update(schema_version=2,planner_variant='legacy_centers',run_id=task['run_id'],input_dir=str(folder/'input'))
        event['response']=decide(event['request'])
        for name,value in [('request.json',event['request']),('response.json',event['response'])]:(folder/name).write_text(json.dumps(value))
        for native in event['captures']:native['fixed_exposure_ev100']=13
        record=dict(fixture.record,schema_version=2,scope=tasks['scope'],manifest_sha256=tasks['manifest_sha256'],expected_runs=2,
            runs=[dict(run_id=task['run_id'],input=task,execution=fixture.raw)])
        report=evaluate(tasks,record,fixture.root,fixture.truth)
        self.assertEqual(report['overall']['runs'],2);self.assertEqual(report['overall']['successes'],1)
        self.assertFalse(report['finished']);self.assertEqual(report['runs'][1]['failure_reason'],'missing_execution')
        event['captures'][0]['fixed_exposure_ev100']=1
        report=evaluate(tasks,record,fixture.root,fixture.truth)
        self.assertEqual(report['overall']['successes'],0);self.assertEqual(report['runs'][0]['failure_reason'],'invalid_execution')
