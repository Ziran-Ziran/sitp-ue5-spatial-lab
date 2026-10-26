import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))
from rendered_replay import CALIBRATION,DEPTH,RENDER_SETTINGS


class RenderedInputTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('rendered_input_report',TOOLS/'rendered_input_report.py')
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        (self.root/'frames').mkdir();(self.root/'depth').mkdir()
        self.sensor=dict(schema_version=1,modality='rgbd_offline_pose_replay',calibration=CALIBRATION,
            depth=DEPTH,render_settings=RENDER_SETTINGS,time_source='replay_source_simulated_seconds',
            timestamp_interpretation='saved engine episode time; not physical sensor sampling or render throughput')
        self.write_sensor()
        (self.root/'timestamps.jsonl').write_text('{"frame_id":0,"t_source_sim_s":0}\n',encoding='utf-8')
        cv2.imwrite(str(self.root/'frames/frame_000000.png'),np.full((240,320,3),123,np.uint8))
        cv2.imwrite(str(self.root/'depth/frame_000000.png'),np.full((240,320),10000,np.uint16))

    def write_sensor(self):
        (self.root/'sensor.json').write_text(json.dumps(self.sensor),encoding='utf-8')

    def test_valid_pair_has_contract_and_depth_statistics(self):
        report=self.module.inspect_sequence(self.root)
        self.assertTrue(report['input_contract_ready'])
        self.assertEqual(report['frames'],1)
        self.assertEqual(report['frames_with_valid_depth'],1)

    def test_empty_depth_is_retained_without_claiming_observable_geometry(self):
        cv2.imwrite(str(self.root/'depth/frame_000000.png'),np.zeros((240,320),np.uint16))
        report=self.module.inspect_sequence(self.root)
        self.assertTrue(report['input_contract_ready'])
        self.assertEqual(report['frames_with_valid_depth'],0)

    def test_truth_or_pose_fields_in_algorithm_metadata_are_rejected(self):
        self.sensor['camera_position_cm']=[12,34,56];self.write_sensor()
        with self.assertRaises(ValueError):self.module.inspect_sequence(self.root)

    def test_truth_file_inside_algorithm_input_is_rejected(self):
        (self.root/'poses.jsonl').write_text('{}',encoding='utf-8')
        with self.assertRaises(ValueError):self.module.inspect_sequence(self.root)

    def test_missing_pair_or_unpaired_extra_is_rejected(self):
        (self.root/'depth/frame_000000.png').unlink()
        with self.assertRaises(ValueError):self.module.inspect_sequence(self.root)

    def test_wrong_depth_dtype_is_rejected(self):
        cv2.imwrite(str(self.root/'depth/frame_000000.png'),np.zeros((240,320),np.uint8))
        with self.assertRaises(ValueError):self.module.inspect_sequence(self.root)

    def test_negative_time_and_noninteger_frame_id_are_rejected(self):
        for row in [dict(frame_id=0,t_source_sim_s=-.5),dict(frame_id=0.0,t_source_sim_s=0)]:
            (self.root/'timestamps.jsonl').write_text(json.dumps(row)+'\n',encoding='utf-8')
            with self.subTest(row=row),self.assertRaises(ValueError):self.module.inspect_sequence(self.root)
