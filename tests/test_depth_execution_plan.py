import copy
from pathlib import Path
import sys
import unittest
import test_pitch_sweep_capture as fixtures

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from depth_execution_plan import build_bundle,validate_bundle,digest_bundle,local_to_world,build_controls,CONDITION


class DepthExecutionPlanTests(unittest.TestCase):
    def setUp(self):
        source=fixtures.PitchSweepCaptureTests();source.setUp();self.addCleanup(source.doCleanups)
        self.capture=source.root
        self.job_binding=dict(render_job_sha256='1'*64,render_status_sha256='2'*64,historical_render_binary_sha256='3'*64)

    def build(self):return build_bundle(self.capture,source_job_binding=self.job_binding)

    def test_geometry_failures_and_approved_routes_both_remain_in_bundle(self):
        bundle=self.build()
        self.assertEqual(len(bundle['runs']),2)
        level,sweep=bundle['runs']
        self.assertEqual(level['gate_status'],'planning_start_unobserved')
        self.assertEqual(level['world_waypoints_cm'],[])
        self.assertEqual(sweep['gate_status'],'approved')
        self.assertEqual(sweep['world_waypoints_cm'][0],[0,0,0])
        self.assertEqual(sweep['world_waypoints_cm'][-1],[300,0,0])
        self.assertEqual(validate_bundle(bundle),bundle)
        self.assertEqual(bundle['manifest_sha256'],self.build()['manifest_sha256'])

    def test_world_pose_conversion_is_explicit_actuation_supervision(self):
        actual=local_to_world([1,2],[100,200,-50],90)
        for a,b in zip(actual,[0,400,-50]):self.assertAlmostEqual(a,b)

    def test_rehashed_missing_run_and_changed_waypoint_are_rejected(self):
        bundle=self.build();bundle['runs'].pop();bundle['manifest_sha256']=digest_bundle(bundle)
        with self.assertRaises(ValueError):validate_bundle(bundle)
        bundle=self.build();bundle['runs'][1]['world_waypoints_cm'][-1][0]+=1
        bundle['manifest_sha256']=digest_bundle(bundle)
        with self.assertRaises(ValueError):validate_bundle(bundle)

    def test_controls_have_distinct_scope_and_no_depth_success_claim(self):
        bundle=build_controls('/Game/SpatialLab/Controls','a'*64)
        self.assertEqual(bundle['execution_scope'],'synthetic_supplied_route_controls')
        self.assertEqual(len(bundle['runs']),5)
        self.assertEqual(bundle['condition'],CONDITION)
        self.assertEqual(validate_bundle(bundle),bundle)
        bundle['execution_scope']='frozen_static_depth_transfer';bundle['manifest_sha256']=digest_bundle(bundle)
        with self.assertRaises(ValueError):validate_bundle(bundle)
