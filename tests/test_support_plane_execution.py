import copy
from pathlib import Path
import sys
import unittest
import test_pitch_sweep_capture as fixtures
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from support_plane_execution import build_bundle,validate_bundle,MODELS,SCOPE
from depth_execution_plan import digest_bundle,validate_bundle as validate_original
from depth_execution_report import evaluate


class SupportPlaneExecutionTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.PitchSweepCaptureTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        self.capture=fixture.root
        self.binding=dict(render_job_sha256='1'*64,render_status_sha256='2'*64,historical_render_binary_sha256='3'*64)

    def build(self):return build_bundle(self.capture,source_job_binding=self.binding)

    def test_both_models_use_same_query_pose_gate_and_explicit_scope(self):
        bundle=self.build();self.assertEqual(bundle['schema_version'],2)
        self.assertEqual(bundle['execution_scope'],SCOPE)
        self.assertEqual([row['mode'] for row in bundle['runs']],list(MODELS))
        for row in bundle['runs']:
            self.assertEqual(row['gate_status'],'approved');self.assertEqual(row['local_goal_m'],[0,3])
            self.assertEqual(row['world_waypoints_cm'][-1],[300,0,0])
        self.assertEqual(bundle['manifest_sha256'],self.build()['manifest_sha256'])
        with self.assertRaises(ValueError):validate_original(bundle)
        with self.assertRaises(ValueError):evaluate(bundle,{})

    def test_rehashed_model_drop_rename_and_world_route_changes_are_rejected(self):
        original=self.build()
        def dropped(b):b['runs'].pop();b['model_diagnostics'].pop()
        def renamed(b):b['runs'][1]['mode']='sweep'
        def moved(b):b['runs'][1]['world_waypoints_cm'][-1][2]+=1
        def unsupported(b):b['model_diagnostics'][1]['support_plane']['status']='refinement_rejected'
        def altered_settings(b):b['settings']['tracking_margin_m']=0
        for mutation in (dropped,renamed,moved,unsupported,altered_settings):
            with self.subTest(mutation=mutation.__name__):
                bundle=copy.deepcopy(original);mutation(bundle);bundle['manifest_sha256']=digest_bundle(bundle)
                with self.assertRaises(ValueError):validate_bundle(bundle)

    def test_physics_scorer_keeps_models_missing_cases_and_feedback_requirements(self):
        from test_depth_execution_report import execution
        bundle=self.build()
        record=dict(condition=bundle['condition'],execution_scope=bundle['execution_scope'],
            manifest_sha256=bundle['manifest_sha256'],map=bundle['map'],
            evidence_kind='ue_game_world_supplied_depth_waypoint_character_movement',executor='CharacterMovement_AddMovementInput',
            position_feedback_source='engine_ground_truth_for_actuation',position_reference='capsule_feet_cm',
            expected_runs=2,attempt_id='12345678-1234-4234-8234-123456789abc',finished=True,
            runs=[dict(run_id=row['run_id'],input=copy.deepcopy(row),execution=execution(row)) for row in bundle['runs']])
        report=evaluate(bundle,record,bundle_validator=validate_bundle)
        self.assertEqual(set(report['by_mode']),set(MODELS));self.assertEqual(report['overall']['successes'],2)
        record['runs'][1]['execution']['waypoint_events']=[]
        report=evaluate(bundle,record,bundle_validator=validate_bundle)
        self.assertEqual(report['runs'][1]['failure_reason'],'invalid_execution')
        record['runs'].pop()
        report=evaluate(bundle,record,bundle_validator=validate_bundle)
        self.assertFalse(report['finished']);self.assertEqual(report['overall']['runs'],2)
        self.assertEqual(report['runs'][1]['failure_reason'],'missing_execution')
