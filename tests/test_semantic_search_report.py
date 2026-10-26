import copy
import importlib.util
from pathlib import Path
import sys
import unittest
import test_semantic_observations as observation_fixtures
import test_semantic_search_tasks as task_fixtures

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))
from semantic_search_tasks import build_bundle,digest_bundle


class SearchReportTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('semantic_search_report',TOOLS/'semantic_search_report.py')
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        fixture=task_fixtures.SearchTaskTests();fixture.setUp()
        self.truth=fixture.source
        self.truth['entities']=[dict(entity_id='observed-chair',category='chair',location_cm=[200,0,0])]
        self.bundle=build_bundle(self.truth,fixture.anchors)
        mission=self.bundle['missions'][0]
        source=observation_fixtures.ObservationStreamTests();source.setUp()
        header=copy.deepcopy(source.header)
        header.update(manifest_sha256=self.bundle['manifest_sha256'],route_source='observation_dependent_engine_navmesh_geometry')
        frame=copy.deepcopy(source.records[2]);frame['run_id']=mission['run_id'];frame['entities'][0]['position_cm']=[200,0,0]
        action=dict(action_id=0,kind='stop',reason='candidate_reached',goal_cm=[200,0,0],entity_position_cm=[200,0,0],
                    entity_id='observed-chair',source_frame_id=0,anchor_index=None)
        state=dict(status='candidate_reached',entity_id='observed-chair',initial_position_cm=[0,0,0],
            final_position_cm=[0,0,0],trajectory_cm=[[0,0,0]],continuous_path_length_cm=0,sim_s=0,wall_s=.1,
            policy_wall_s=.01,navigation_requests=0,collisions=0,
            policy_trace=[dict(frame_id=0,t_sim_s=0,feedback=dict(action_id=None,state='none'),action=action,policy_wall_s=.01)])
        self.execution=dict(finished=True,map=header['map'],manifest_sha256=header['manifest_sha256'],
            attempt_id=header['attempt_id'],condition=header['condition'],
            evidence_kind='ue_game_world_observation_dependent_search',units='centimeters',
            position_reference='capsule_feet',geometry_source='engine_navmesh_exploration_and_paths',
            runs=[dict(run_id=mission['run_id'],policy=mission['policy'],mission=mission,execution=state)])
        self.records=[header,dict(record_type='episode_start',run_id=mission['run_id']),frame,
            dict(record_type='episode_end',run_id=mission['run_id'],status='candidate_reached',frames=1),
            dict(record_type='stream_end',finished=True,episodes=1,frames=1)]

    def score(self):return self.module.evaluate(self.bundle,self.truth,self.execution,self.records)

    def test_independent_category_and_position_score_with_missing_runs_retained(self):
        result=self.score()
        self.assertFalse(result['finished'])
        self.assertTrue(result['runs'][0]['success'])
        self.assertEqual(result['runs'][1]['failure_reason'],'missing_execution')
        self.assertEqual(result['by_policy']['temporal_memory']['episodes'],1)

    def test_claimed_candidate_far_from_truth_is_failure(self):
        self.execution['runs'][0]['execution']['final_position_cm']=[999,0,0]
        self.records[2].update(observer_position_cm=[999,0,0],sensor_eye_cm=[999,0,150])
        self.execution['runs'][0]['execution']['initial_position_cm']=[999,0,0]
        self.assertFalse(self.score()['runs'][0]['success'])

    def test_hidden_or_future_candidate_cannot_succeed(self):
        state=self.execution['runs'][0]['execution']
        state['policy_trace'][0]['action']['source_frame_id']=1
        self.assertEqual(self.score()['runs'][0]['failure_reason'],'invalid_observation_provenance')

    def test_changed_mission_or_truth_is_rejected(self):
        self.truth['entities'][0]['location_cm'][0]=201
        with self.assertRaises(ValueError):self.score()

    def test_unknown_collisions_are_not_zero(self):
        self.execution['runs'][0]['execution']['collisions']=None
        result=self.score()
        self.assertEqual(result['by_policy']['frame_only']['collision_observed_episodes'],0)

    def test_observed_candidate_with_forged_policy_feedback_is_rejected(self):
        state=self.execution['runs'][0]['execution']
        state['policy_trace'][0]['feedback']['action_id']=77
        self.assertEqual(self.score()['runs'][0]['failure_reason'],'invalid_policy_trace')

    def test_oracle_success_status_cannot_bypass_search_provenance(self):
        self.execution['runs'][0]['execution']['status']='arrived'
        self.records[-2]['status']='arrived'
        self.assertFalse(self.score()['runs'][0]['success'])

    def test_wrong_execution_evidence_or_route_source_is_rejected(self):
        self.execution['evidence_kind']='editor_navmesh_planning_not_execution'
        with self.assertRaises(ValueError):self.score()
        self.execution['evidence_kind']='ue_game_world_observation_dependent_search'
        self.records[0]['route_source']='global_engine_oracle_selected_routes'
        with self.assertRaises(ValueError):self.score()

    def test_rehashed_bundle_cannot_drop_a_frozen_policy_or_change_budgets(self):
        for change in ('drop','budget'):
            with self.subTest(change=change):
                changed=copy.deepcopy(self.bundle)
                if change=='drop':changed['missions'].pop()
                else:changed['missions'][0]['max_sim_s']=30
                changed['manifest_sha256']=digest_bundle(changed)
                execution=copy.deepcopy(self.execution);execution['manifest_sha256']=changed['manifest_sha256']
                records=copy.deepcopy(self.records);records[0]['manifest_sha256']=changed['manifest_sha256']
                with self.assertRaises(ValueError):self.module.evaluate(changed,self.truth,execution,records)
