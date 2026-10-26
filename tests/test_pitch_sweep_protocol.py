import copy
import json
from pathlib import Path
import sys
import unittest
import test_rendered_replay as fixtures

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from pitch_sweep_protocol import build_sweep,validate_sweep,sweep_digest,view_metadata,rotation_matches,VIEWS


class PitchSweepProtocolTests(unittest.TestCase):
    def setUp(self):
        source=fixtures.RenderedReplayTests();source.setUp();self.base=source.plan()

    def test_fixed_views_wrap_original_source_and_failures(self):
        result=build_sweep(self.base)
        self.assertEqual(result['base_plan'],self.base)
        self.assertEqual(result['views'],VIEWS)
        self.assertEqual(validate_sweep(result),result)

    def test_rehashed_wrong_angles_and_changed_base_are_rejected(self):
        result=build_sweep(self.base);result['views'][1]['pitch_down_deg']=30
        result['sweep_sha256']=sweep_digest(result)
        with self.assertRaises(ValueError):validate_sweep(result)
        result=build_sweep(self.base);result['base_plan']['map']='/Game/SpatialLab/Other'
        result['sweep_sha256']=sweep_digest(result)
        with self.assertRaises(ValueError):validate_sweep(result)

    def test_relative_extrinsics_contain_no_global_pose_or_run(self):
        metadata=view_metadata(self.base,45)
        self.assertEqual(metadata['relative_pitch_down_deg'],45)
        for word in ('camera_position_cm','/Game/','run-a','source_attempt_id'):
            self.assertNotIn(word,json.dumps(metadata))
        with self.assertRaises(ValueError):view_metadata(self.base,46)

    def test_nadir_euler_equivalence_is_compared_as_rotation(self):
        self.assertTrue(rotation_matches([-90,30,0],[-90,0,30]))
        self.assertFalse(rotation_matches([-90,30,0],[-90,30,20]))
        self.assertTrue(rotation_matches([0,180,0],[0,-180,0]))
