"""Synthetic geometry and ground-truth isolation checks for COLMAP dense fusion."""
import json
import contextlib
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import cv2
from scipy.spatial.transform import Rotation

import colmap_depth_poses as cp
import dense_map
from dense_map import ground_truth_pose, backproject


class GeometryTests(unittest.TestCase):
    def test_metric_scale_uses_optical_axis_and_rejects_outliers(self):
        # Lateral offsets must not change the scale, including off-axis points.
        depths = np.linspace(1, 15, 101)
        measured = depths * 2.3
        measured[::10] *= 7
        scale, stats = cp.fit_depth_scale(depths, measured)
        self.assertAlmostEqual(scale, 2.3, places=10)
        self.assertGreater(stats['inlier_fraction'], .85)

    def test_scale_rejects_insufficient_geometry(self):
        with self.assertRaises(ValueError):
            cp.fit_depth_scale(np.zeros(100), np.ones(100))

    def test_pose_roundtrip_preserves_cv_projection(self):
        world_from_cv = Rotation.from_euler('xyz', [23, -37, 81], degrees=True).as_matrix()
        centre = np.array([3., 4., -2.])
        cv_point = np.array([1., -2., 5.])
        calibration = dict(fx=100., fy=100., cx=50., cy=50., w=100, h=100)
        record = cp.pose_record(17, world_from_cv, centre, calibration)
        transform = ground_truth_pose(json.loads(json.dumps(record)))
        expected = world_from_cv @ cv_point + centre
        depth = np.zeros((100, 100), np.uint16)
        depth[10, 70] = 5000
        points, _ = backproject(depth, record, 1)
        actual = points @ transform[:3, :3].T + transform[:3, 3]
        np.testing.assert_allclose(actual[0], expected, atol=1e-6)
        self.assertAlmostEqual(np.linalg.det(transform[:3, :3]), 1.)

    def test_calibration_load_does_not_need_truth_file(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            (session / 'meta.json').write_text(json.dumps(dict(
                width=1280, height=720, fov_degrees_initial=90)), encoding='utf-8')
            intrinsics = cp.load_calibration(session)
            self.assertAlmostEqual(intrinsics['fx'], 640.)
            self.assertAlmostEqual(intrinsics['fy'], 640.)
            self.assertFalse((session / 'poses.jsonl').exists())

    @unittest.skipUnless(Path('output/ue5-eval/colmap-city-lap2/best').exists(),
                         'optional real-session isolation regression')
    def test_real_colmap_fuses_without_any_truth_files(self):
        model = Path('output/ue5-eval/colmap-city-lap2/best').resolve()
        source = Path('output/ue5-datasets/city-lap2').resolve()
        # Hard links keep the real depth bytes, while the fixture deliberately has
        # no truth poses, no start_pose, no labels and no RGB frames.
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / 'session'
            session.mkdir()
            (session / 'depth').mkdir()
            meta = json.loads((source / 'meta.json').read_text(encoding='utf-8'))
            (session / 'meta.json').write_text(json.dumps({k: meta[k] for k in
                ['width', 'height', 'fov_degrees_initial']}), encoding='utf-8')
            for depth in (source / 'depth').glob('frame_*.png'):
                os.link(depth, session / 'depth' / depth.name)
            out = Path(directory) / 'result'
            argv = ['dense_map.py', str(session), '--poses', 'colmap',
                    '--model', str(model), '--out', str(out), '--stride', '96', '--fuse-only']
            with patch('sys.argv', argv), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(dense_map.main(), 0)
            report = json.loads((out / 'dense_native.json').read_text(encoding='utf-8'))
            self.assertEqual(report['frames_used'], 475)
            self.assertFalse(report['ground_truth_used_for_fusion'])
            self.assertTrue((out / 'dense_native.npz').exists())
            self.assertFalse((out / 'dense_eval.json').exists())

    def test_label_scoring_matches_direct_definition(self):
        random = np.random.default_rng(3)
        points = random.normal(size=(100, 3))
        landmarks = random.normal(size=(10, 3))
        normals = random.normal(size=(10, 3))
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)
        distances = np.linalg.norm(points[:, None] - landmarks[None], axis=2)
        nearest = distances.argmin(axis=1)
        near = distances.min(axis=1) < 2.
        perpendicular = np.abs(((points - landmarks[nearest]) * normals[nearest]).sum(axis=1))[near]
        report = dense_map.label_metrics(points, landmarks, normals)
        self.assertEqual(report['perpendicular_to_label_plane_m']['median'],
                         round(float(np.median(perpendicular)), 3))
        self.assertEqual(report['label_recall_0_5_m'],
                         round(float((distances.min(axis=0) < .5).mean()), 3))

    def test_oracle_control_uses_exact_reference_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            session=Path(directory)/'synthetic'
            session.mkdir(); (session/'depth').mkdir()
            calibration=dict(fx=64.,fy=64.,cx=64.,cy=36.,w=128,h=72,fov=90.)
            poses=[dict(i=i,loc=[float(i),0.,100.],quat=[0.,0.,0.,1.],**calibration) for i in range(24)]
            (session/'poses.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in poses),encoding='utf-8')
            (session/'meta.json').write_text(json.dumps(dict(start_pose=dict(loc=[0.,0.,100.]))),encoding='utf-8')
            (session/'landmarks.json').write_text(json.dumps(dict(points=[[200.,0.,100.],[200.,100.,100.]])),encoding='utf-8')
            for i in range(24):
                cv2.imwrite(str(session/'depth'/f'frame_{i:06d}.png'),np.full((72,128),2000,np.uint16))
            indices=list(range(0,24,2))
            reference=session/'reference.json'
            reference.write_text(json.dumps(dict(pose_source='colmap',used_frame_indices=indices)),encoding='utf-8')
            out=session/'result'
            with patch('sys.argv',['dense_map.py',str(session),'--poses','gt','--out',str(out),
                                  '--frame-indices',str(reference)]), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(dense_map.main(),0)
            report=json.loads((out/'dense_eval.json').read_text())
            self.assertEqual(report['used_frame_indices'],indices)
            self.assertEqual(report['coverage'],.5)
            self.assertTrue(report['ground_truth_used_for_fusion'])
            self.assertEqual(report['matched_frame_control']['pose_source'],'colmap')

    def test_reference_selection_cannot_be_used_by_estimated_fusion(self):
        with patch('sys.argv',['dense_map.py','unused-session','--poses','colmap',
                              '--frame-indices','unused.json','--out','unused-output']), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                dense_map.main()
        self.assertEqual(error.exception.code,2)


if __name__ == '__main__':
    unittest.main()
