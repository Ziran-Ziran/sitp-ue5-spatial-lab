from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import test_pitch_sweep_input as fixtures
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from support_plane_replay import evaluate_sequence,geometry_sources
from support_plane_grid import MODEL


class SupportPlaneReplayTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.PitchSweepInputTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        self.sequence=fixture.sequence

    def test_independent_floor_pixels_both_models_without_truth_access(self):
        original=Path.open
        def guarded(path,*args,**kwargs):
            if 'evaluation' in path.parts:raise AssertionError('truth accessed')
            return original(path,*args,**kwargs)
        with patch.object(Path,'open',guarded):report=evaluate_sequence(self.sequence)
        frame=report['frames'][0]
        for model in ('horizontal_v1',MODEL):
            self.assertEqual(frame[model]['route']['continuous_gate']['status'],'approved')
        self.assertEqual(frame[MODEL]['support_plane']['status'],'supported')
        self.assertIn('support_plane_grid.py',report['geometry_source_sha256'])
        self.assertIn('continuous_depth_path.py',report['geometry_source_sha256'])
        self.assertNotIn('depth_execution_plan.py',report['geometry_source_sha256'])

    def test_changed_geometry_source_during_replay_is_rejected(self):
        sources=geometry_sources()
        with patch('support_plane_replay.geometry_sources',side_effect=[sources,{}]):
            with self.assertRaisesRegex(ValueError,'geometry source changed'):evaluate_sequence(self.sequence)

    def test_privileged_camera_pose_in_pixel_contract_is_rejected(self):
        import json
        path=self.sequence/'nadir/sensor.json';sensor=json.loads(path.read_text())
        sensor['camera_position_cm']=[0,0,150];path.write_text(json.dumps(sensor))
        with self.assertRaises(ValueError):evaluate_sequence(self.sequence)
