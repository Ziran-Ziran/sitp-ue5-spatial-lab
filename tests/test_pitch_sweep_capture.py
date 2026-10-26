import json
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np
import test_pitch_sweep_input as fixtures

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from pitch_sweep_protocol import build_sweep,VIEWS
from rendered_replay import digest
from rendered_input_report import file_hash
from pitch_sweep_capture_report import inspect_sweep_capture


class PitchSweepCaptureTests(unittest.TestCase):
    def setUp(self):
        fixture=fixtures.PitchSweepInputTests();fixture.setUp();self.addCleanup(fixture.temp.cleanup)
        self.root=fixture.root;self.sequence=fixture.sequence
        (self.root/'input').mkdir();self.sequence.rename(self.root/'input/sequence_000')
        self.sequence=self.root/'input/sequence_000'
        base=fixture.base;episode=base['episodes'][0]
        episode['source_frames']=1;episode['frames']=episode['frames'][:1];base['plan_sha256']=digest(base)
        self.plan=build_sweep(base)
        (self.root/'evaluation/sweep-plan.json').write_text(json.dumps(self.plan),encoding='utf-8')
        views={};poses=[]
        for view in VIEWS:
            name=view['name'];directory=self.sequence/name
            frame=dict(episode['frames'][0],camera_rotation_deg=[-view['pitch_down_deg'],0,0])
            poses.append(dict(sequence='sequence_000',run_id=episode['run_id'],view=name,**frame,
                actual_camera_position_cm=frame['camera_position_cm'],actual_camera_rotation_deg=frame['camera_rotation_deg']))
            views[name]=[dict(captured=True,frame_id=0,width=320,height=240,
                fixed_exposure_ev100=base['render_settings']['fixed_exposure_ev100'],
                camera_position_cm=frame['camera_position_cm'],camera_rotation_deg=frame['camera_rotation_deg'],
                color_file=f'input/sequence_000/{name}/frames/frame_000000.png',
                depth_file=f'input/sequence_000/{name}/depth/frame_000000.png',
                rgb_sha256=file_hash(directory/'frames/frame_000000.png'),depth_sha256=file_hash(directory/'depth/frame_000000.png'))]
        (self.root/'evaluation/camera_poses.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in poses),encoding='utf-8')
        self.index=dict(evidence_kind='rendered_static_pitch_sweep',finished=True,sweep_sha256=self.plan['sweep_sha256'],
            **{key:base[key] for key in ('source_attempt_id','source_stream_sha256','map','source_provenance','render_settings')},
            sequences=[dict(sequence='sequence_000',run_id=episode['run_id'],source_status=episode['source_status'],
                source_frames=1,expected_frames=1,views=views)])
        self.save()

    def save(self):(self.root/'capture-index.json').write_text(json.dumps(self.index),encoding='utf-8')

    def test_all_view_images_and_actual_transforms_bind_to_source(self):
        self.assertEqual(inspect_sweep_capture(self.root)['image_pairs'],3)

    def test_swapped_view_image_is_rejected(self):
        cv2.imwrite(str(self.sequence/'nadir/depth/frame_000000.png'),np.zeros((240,320),np.uint16))
        with self.assertRaises(ValueError):inspect_sweep_capture(self.root)

    def test_wrong_native_pitch_and_different_source_attempt_are_rejected(self):
        self.index['sequences'][0]['views']['down45'][0]['camera_rotation_deg']=[45,0,0];self.save()
        with self.assertRaises(ValueError):inspect_sweep_capture(self.root)
        self.index['source_attempt_id']='other';self.save()
        with self.assertRaises(ValueError):inspect_sweep_capture(self.root)
