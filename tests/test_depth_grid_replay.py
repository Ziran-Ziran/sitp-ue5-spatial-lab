from pathlib import Path
import json
import sys
import tempfile
import unittest
from unittest.mock import patch
import cv2
import numpy as np
import test_local_depth_grid as geometry

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from rendered_replay import CALIBRATION,DEPTH,RENDER_SETTINGS
from local_depth_grid import GridConfig
from depth_grid_replay import evaluate_sequence


class DepthGridReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.sequence=self.root/'sequence_000';self.sequence.mkdir()
        for name in ('frames','depth'):(self.sequence/name).mkdir()
        sensor=dict(schema_version=1,modality='rgbd_offline_pose_replay',calibration=CALIBRATION,
            depth=DEPTH,render_settings=RENDER_SETTINGS,time_source='replay_source_simulated_seconds',
            timestamp_interpretation='saved engine episode time; not physical sensor sampling or render throughput')
        (self.sequence/'sensor.json').write_text(json.dumps(sensor),encoding='utf-8')
        (self.sequence/'timestamps.jsonl').write_text(json.dumps(dict(frame_id=0,t_source_sim_s=0))+'\n',encoding='utf-8')
        cv2.imwrite(str(self.sequence/'depth/frame_000000.png'),geometry.floor_depth())
        cv2.imwrite(str(self.sequence/'frames/frame_000000.png'),np.zeros((240,320,3),np.uint8))
        (self.root/'evaluation').mkdir();(self.root/'evaluation/poses.jsonl').write_text('DO NOT READ',encoding='utf-8')

    def test_true_agent_start_keeps_sensing_gap_failure(self):
        result=evaluate_sequence(self.sequence,GridConfig(),start=(0,0),goal=(0,3))
        self.assertEqual(result['frames'][0]['planning']['status'],'start_unobserved')
        self.assertEqual(result['frames'][0]['planning']['cells'],[])
        self.assertEqual(result['path_found_frames'],0)
        self.assertTrue(result['source_files_sha256'])

    def test_supported_patch_can_produce_a_geometric_path_without_truth_access(self):
        original=Path.open
        opened=[]
        def guarded(path,*args,**kwargs):
            opened.append(str(path))
            if 'evaluation' in path.parts:raise AssertionError('truth read')
            return original(path,*args,**kwargs)
        with patch.object(Path,'open',guarded):
            result=evaluate_sequence(self.sequence,GridConfig(),start=(0,3),goal=(0,5))
        self.assertEqual(result['frames'][0]['planning']['status'],'path_found')
        self.assertGreater(result['frames'][0]['planning']['path_length_m'],0)
        self.assertFalse(any('evaluation' in path for path in opened))

    def test_privileged_files_and_bad_timestamps_are_rejected(self):
        (self.sequence/'poses.jsonl').write_text('{}',encoding='utf-8')
        with self.assertRaises(ValueError):evaluate_sequence(self.sequence,GridConfig())
        (self.sequence/'poses.jsonl').unlink()
        (self.sequence/'timestamps.jsonl').write_text('{"frame_id":0,"t_source_sim_s":-1}\n',encoding='utf-8')
        with self.assertRaises(ValueError):evaluate_sequence(self.sequence,GridConfig())
