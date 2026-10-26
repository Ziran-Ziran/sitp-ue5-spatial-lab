import copy
from pathlib import Path
import sys
import math
import unittest
import cv2
import numpy as np
import test_visual_marker_depth_policy as fixtures
import test_city_marker_tasks as city_fixture
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from marker_goal_horizon import POLICIES
from online_depth_policy import write_observation,CALIBRATION
from visual_marker_depth_policy import decide
from visual_marker_report import without_cpu
from visual_marker_tasks import build_tasks,validate_tasks
from online_depth_tasks import digest


class MarkerHorizonPolicyTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.MarkerDepthPolicyTests();fixture.setUp();self.addCleanup(fixture.doCleanups);self.fixture=fixture

    def request(self,policy):
        return dict(self.fixture.request,schema_version=3,planner_variant='legacy_centers',goal_horizon_policy=policy)

    def test_one_meter_observed_floor_can_move_when_three_meter_goal_is_unknown(self):
        fixture=self.fixture;rows=np.indices((240,320))[0]
        for name,pitch in (('level',0),('down45',45),('nadir',90)):
            path=fixture.root/name/'depth.png';depth=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
            angle=math.radians(pitch);forward=depth.astype(float)/1000*(math.cos(angle)-(rows-CALIBRATION['cy'])/CALIBRATION['fy']*math.sin(angle))
            depth[forward>1.8]=0;cv2.imwrite(str(path),depth)
        fixture.marker((0,0,255),4000)
        old=decide(self.request(POLICIES[0]));near=decide(self.request(POLICIES[1]))
        self.assertEqual(old['action'],'reject');self.assertEqual(near['action'],'move')
        self.assertLessEqual(math.hypot(*near['geometry_goal_right_forward_m']),1.000000001)

    def test_three_meter_schema_three_preserves_all_old_decision_fields(self):
        self.fixture.marker((0,0,255),2000)
        original=without_cpu(decide(dict(self.fixture.request,schema_version=2,planner_variant='legacy_centers')))
        current=without_cpu(decide(self.request(POLICIES[0])));current['schema_version']=2;current.pop('goal_horizon_policy')
        self.assertEqual(current,original)
        with self.assertRaises(ValueError):decide(dict(self.request(POLICIES[1]),goal_cm=[0,0,0]))
        with self.assertRaises(ValueError):decide(dict(self.fixture.request,schema_version=3,goal_horizon_policy=POLICIES[1]))

    def test_complete_neutral_and_city_tasks_bind_one_horizon_without_goal_truth(self):
        city=city_fixture.CityMarkerTasksTests();city.setUp()
        from city_marker_tasks import build_tasks as city_tasks
        for policy in POLICIES:
            for tasks in (build_tasks('/Game/SpatialLab/Markers','a'*64,planner_variant='legacy_centers',goal_horizon_policy=policy),
                    city_tasks('/Game/SpatialLab/CityMarkers','d'*64,city.source,city.binding,'legacy_centers',goal_horizon_policy=policy)):
                self.assertEqual(tasks['schema_version'],3);self.assertEqual(validate_tasks(tasks),tasks)
                self.assertTrue(all(row['goal_horizon_policy']==policy and 'goal_cm' not in row for row in tasks['runs']))
                tasks['runs'][0]['goal_horizon_policy']='changed';tasks['manifest_sha256']=digest(tasks)
                with self.assertRaises(ValueError):validate_tasks(tasks)

    def test_native_pixel_audit_requires_same_horizon_and_preserves_truth_scoring(self):
        import json
        import test_visual_marker_report as evidence
        from visual_marker_report import audit_execution,evaluate
        fixture=evidence.VisualMarkerReportTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        tasks=build_tasks('/Game/SpatialLab/Markers','a'*64,planner_variant='legacy_centers',goal_horizon_policy=POLICIES[1])
        task=tasks['runs'][8];event=fixture.raw['actions'][0];folder=Path(event['folder'])
        event['request'].update(schema_version=3,planner_variant='legacy_centers',goal_horizon_policy=POLICIES[1])
        event['response']=decide(event['request'])
        for name,value in (('request.json',event['request']),('response.json',event['response'])):(folder/name).write_text(json.dumps(value),encoding='utf-8')
        self.assertTrue(audit_execution(task,fixture.raw,fixture.root,fixture.attempt,8)['candidate_reached'])
        record=dict(fixture.record,schema_version=3,manifest_sha256=tasks['manifest_sha256'],runs=[dict(run_id='immediate',input=task,execution=fixture.raw)])
        report=evaluate(tasks,record,fixture.root,fixture.truth)
        self.assertEqual(report['goal_horizon_policy'],POLICIES[1]);self.assertEqual(report['overall']['successes'],1)
        task=dict(task,goal_horizon_policy=POLICIES[0])
        with self.assertRaises(ValueError):audit_execution(task,fixture.raw,fixture.root,fixture.attempt,8)
