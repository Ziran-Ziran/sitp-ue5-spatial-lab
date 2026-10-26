import copy
import importlib.util
import json
from pathlib import Path
import sys
import unittest
import test_semantic_observations as fixtures

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))


class RenderedReplayTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('rendered_replay',TOOLS/'rendered_replay.py')
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        fixture=fixtures.ObservationStreamTests();fixture.setUp()
        self.records=fixture.records
        self.records[0]['route_source']='observation_dependent_engine_navmesh_geometry'
        for index,feet in enumerate([[0,0,0],[0,100,0],[100,100,0]]):
            self.records[index+2]['observer_position_cm']=feet
            self.records[index+2]['sensor_eye_cm']=[feet[0],feet[1],150]

    def plan(self,count=4):return self.module.build_plan(self.records,'b'*64,frames_per_episode=count,
        source_provenance=dict(map_sha256='c'*64,job_sha256='d'*64,execution_sha256='e'*64))

    def test_render_plan_keeps_source_attempt_but_no_entity_catalog(self):
        plan=self.plan()
        self.assertEqual(plan['source_attempt_id'],self.records[0]['attempt_id'])
        self.assertEqual(len(plan['episodes'][0]['frames']),3)
        self.assertNotIn('observed-chair',json.dumps(plan))
        self.assertNotIn('entities',plan)
        self.assertEqual(self.module.validate_plan(plan),plan)

    def test_heading_is_causal_past_motion_not_target_or_future(self):
        first=self.plan()['episodes'][0]['frames']
        self.assertEqual([frame['camera_rotation_deg'][1] for frame in first],[0,90,0])
        self.records[4]['observer_position_cm']=[-100,100,0]
        self.records[4]['sensor_eye_cm']=[-100,100,150]
        changed=self.plan()['episodes'][0]['frames']
        self.assertEqual(changed[0],first[0])
        self.assertEqual(changed[1],first[1])

    def test_fixed_uniform_selection_includes_endpoints(self):
        frames=self.plan(2)['episodes'][0]['frames']
        self.assertEqual([row['source_frame_id'] for row in frames],[0,2])
        self.assertEqual([row['frame_id'] for row in frames],[0,1])

    def test_algorithm_metadata_has_no_engine_pose_map_run_or_labels(self):
        metadata=self.module.algorithm_metadata(self.plan())
        self.assertEqual(metadata['calibration']['w'],320)
        self.assertEqual(metadata['calibration']['fx'],160)
        for value in ('/Game/','run-a','camera_position_cm','observed-chair','source_attempt_id'):
            self.assertNotIn(value,json.dumps(metadata))
        self.assertEqual(metadata['time_source'],'replay_source_simulated_seconds')

    def test_changed_camera_or_calibration_fails_frozen_digest(self):
        plan=self.plan();plan['episodes'][0]['frames'][0]['camera_position_cm'][0]=10
        with self.assertRaises(ValueError):self.module.validate_plan(plan)

    def test_zero_frame_failure_episode_is_not_silently_dropped(self):
        self.records[-1:-1]=[dict(record_type='episode_start',run_id='failed'),
            dict(record_type='episode_end',run_id='failed',status='spawn_blocked',frames=0)]
        self.records[-1]['episodes']=2
        plan=self.plan()
        self.assertEqual(plan['episodes'][1]['source_status'],'spawn_blocked')
        self.assertEqual(plan['episodes'][1]['frames'],[])

    def test_non_search_or_unbounded_capture_request_is_rejected(self):
        self.records[0]['route_source']='global_engine_oracle_selected_routes'
        with self.assertRaises(ValueError):self.plan()
        self.records[0]['route_source']='observation_dependent_engine_navmesh_geometry'
        with self.assertRaises(ValueError):self.plan(10000)

    def test_rehashed_invalid_attempt_map_or_negative_time_is_rejected(self):
        for key,value in [('source_attempt_id','not-an-attempt'),('map','/Game/SpatialLab/../../Elsewhere')]:
            plan=self.plan();plan[key]=value;plan['plan_sha256']=self.module.digest(plan)
            with self.subTest(key=key),self.assertRaises(ValueError):self.module.validate_plan(plan)
        plan=self.plan();plan['episodes'][0]['frames'][0]['t_source_sim_s']=-.5
        plan['plan_sha256']=self.module.digest(plan)
        with self.assertRaises(ValueError):self.module.validate_plan(plan)
