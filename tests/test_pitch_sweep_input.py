import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import test_depth_pitch_sweep as geometry
import test_rendered_replay as fixtures

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from pitch_sweep_protocol import VIEWS,view_metadata
from pitch_sweep_report import inspect_sweep_sequence,evaluate_sweep_sequence


class PitchSweepInputTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.sequence=self.root/'sequence_000';self.sequence.mkdir()
        base=fixtures.RenderedReplayTests();base.setUp();self.base=base.plan()
        for view in VIEWS:
            directory=self.sequence/view['name'];directory.mkdir()
            for name in ('frames','depth'):(directory/name).mkdir()
            (directory/'sensor.json').write_text(json.dumps(view_metadata(self.base,view['pitch_down_deg'])),encoding='utf-8')
            (directory/'timestamps.jsonl').write_text('{"frame_id":0,"t_source_sim_s":0}\n',encoding='utf-8')
            cv2.imwrite(str(directory/'frames/frame_000000.png'),np.zeros((240,320,3),np.uint8))
            cv2.imwrite(str(directory/'depth/frame_000000.png'),geometry.floor_view(view['pitch_down_deg']))
        (self.root/'evaluation').mkdir();(self.root/'evaluation/poses.jsonl').write_text('must not read',encoding='utf-8')

    def test_aligned_views_plan_without_evaluation_access(self):
        original=Path.open
        def guarded(path,*args,**kwargs):
            if 'evaluation' in path.parts:raise AssertionError('truth read')
            return original(path,*args,**kwargs)
        with patch.object(Path,'open',guarded):
            report=evaluate_sweep_sequence(self.sequence)
        self.assertEqual(report['frames'][0]['level']['planning']['status'],'start_unobserved')
        self.assertEqual(report['frames'][0]['sweep']['planning']['status'],'path_found')

    def test_wrong_relative_pitch_or_privileged_field_is_rejected(self):
        path=self.sequence/'down45/sensor.json';sensor=json.loads(path.read_text())
        sensor['relative_pitch_down_deg']=30;path.write_text(json.dumps(sensor),encoding='utf-8')
        with self.assertRaises(ValueError):inspect_sweep_sequence(self.sequence)
        sensor['relative_pitch_down_deg']=45;sensor['camera_position_cm']=[0,0,150]
        path.write_text(json.dumps(sensor),encoding='utf-8')
        with self.assertRaises(ValueError):inspect_sweep_sequence(self.sequence)

    def test_monotonic_but_unaligned_view_times_are_rejected(self):
        (self.sequence/'down45/timestamps.jsonl').write_text('{"frame_id":0,"t_source_sim_s":1}\n',encoding='utf-8')
        with self.assertRaises(ValueError):inspect_sweep_sequence(self.sequence)

    def test_missing_view_is_rejected_instead_of_assumed_free(self):
        (self.sequence/'nadir').rename(self.sequence/'other')
        with self.assertRaises(ValueError):inspect_sweep_sequence(self.sequence)
