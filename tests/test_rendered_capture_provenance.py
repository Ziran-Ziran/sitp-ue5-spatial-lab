import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import cv2
import numpy as np
import test_rendered_replay as fixtures

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))
from rendered_replay import algorithm_metadata
from rendered_input_report import file_hash,inspect_capture


class CaptureProvenanceTests(unittest.TestCase):
    def setUp(self):
        source=fixtures.RenderedReplayTests();source.setUp();self.plan=source.plan()
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        evaluation=self.root/'evaluation';evaluation.mkdir()
        self.sequence=self.root/'input/sequence_000';self.sequence.mkdir(parents=True)
        (self.sequence/'frames').mkdir();(self.sequence/'depth').mkdir()
        (self.sequence/'sensor.json').write_text(json.dumps(algorithm_metadata(self.plan)),encoding='utf-8')
        (evaluation/'replay-plan.json').write_text(json.dumps(self.plan),encoding='utf-8')
        frames=self.plan['episodes'][0]['frames']
        self.timestamps=[dict(frame_id=f['frame_id'],t_source_sim_s=f['t_source_sim_s']) for f in frames]
        self.write_times()
        poses=[];native=[]
        for frame in frames:
            name=f"frame_{frame['frame_id']:06d}.png"
            rgb=self.sequence/'frames'/name;depth=self.sequence/'depth'/name
            cv2.imwrite(str(rgb),np.full((240,320,3),123,np.uint8))
            cv2.imwrite(str(depth),np.full((240,320),10000,np.uint16))
            poses.append(dict(sequence='sequence_000',run_id='run-a',**frame,
                actual_camera_position_cm=frame['camera_position_cm'],actual_camera_rotation_deg=frame['camera_rotation_deg']))
            native.append(dict(captured=True,frame_id=frame['frame_id'],width=320,height=240,
                fixed_exposure_ev100=self.plan['render_settings']['fixed_exposure_ev100'],
                camera_position_cm=frame['camera_position_cm'],camera_rotation_deg=frame['camera_rotation_deg'],
                color_file='input/sequence_000/frames/'+name,depth_file='input/sequence_000/depth/'+name,
                rgb_sha256=file_hash(rgb),depth_sha256=file_hash(depth)))
        (evaluation/'camera_poses.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in poses),encoding='utf-8')
        episode=self.plan['episodes'][0]
        self.index=dict(schema_version=1,evidence_kind='rendered_static_pose_replay',finished=True,
            plan_sha256=self.plan['plan_sha256'],source_attempt_id=self.plan['source_attempt_id'],
            source_stream_sha256=self.plan['source_stream_sha256'],map=self.plan['map'],sequences=[dict(
                sequence='sequence_000',run_id='run-a',source_status=episode['source_status'],
                source_frames=episode['source_frames'],expected_frames=len(frames),frames=native)])
        self.index.update(source_provenance=self.plan['source_provenance'],render_settings=self.plan['render_settings'])
        self.write_index()

    def write_index(self):(self.root/'capture-index.json').write_text(json.dumps(self.index),encoding='utf-8')
    def write_times(self):(self.sequence/'timestamps.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in self.timestamps),encoding='utf-8')

    def test_full_provenance_links_pass(self):self.assertTrue(inspect_capture(self.root)['input_contract_ready'])

    def test_monotonic_but_wrong_source_time_is_rejected(self):
        self.timestamps[-1]['t_source_sim_s']=12345;self.write_times()
        with self.assertRaises(ValueError):inspect_capture(self.root)

    def test_missing_plan_and_false_native_capture_are_rejected(self):
        self.index['sequences'][0]['frames'][0]['captured']=False;self.write_index()
        with self.assertRaises(ValueError):inspect_capture(self.root)
        (self.root/'evaluation/replay-plan.json').unlink()
        with self.assertRaises((ValueError,FileNotFoundError)):inspect_capture(self.root)

    def test_swapped_valid_png_is_rejected_by_collector_hash(self):
        cv2.imwrite(str(self.sequence/'frames/frame_000000.png'),np.full((240,320,3),99,np.uint8))
        with self.assertRaises(ValueError):inspect_capture(self.root)

    def test_actual_pose_or_attempt_mismatch_is_rejected(self):
        self.index['source_attempt_id']='87654321-1234-4234-8234-123456789abc';self.write_index()
        with self.assertRaises(ValueError):inspect_capture(self.root)
