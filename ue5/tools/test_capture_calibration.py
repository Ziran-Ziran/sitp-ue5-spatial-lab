"""Independent calibration must never require the ground-truth pose file."""
import json
from pathlib import Path
import tempfile
import unittest
import contextlib
import io
from unittest.mock import patch

import cv2
import numpy as np

from colmap_depth_poses import load_calibration
from run_colmap import read_intrinsics


class CalibrationTests(unittest.TestCase):
    def fixture(self, directory, calibration=None, **extra):
        session = Path(directory)
        meta = dict(width=1280, height=720, fov_degrees_initial=90,
                    calibration_constant=True, **extra)
        if calibration is not None:
            meta['calibration'] = calibration
        (session / 'meta.json').write_text(json.dumps(meta), encoding='utf-8')
        return session

    def test_sfm_reads_metadata_without_truth(self):
        with tempfile.TemporaryDirectory() as d:
            session = self.fixture(d)
            fx, fy, cx, cy, w, h = read_intrinsics(session)
            self.assertAlmostEqual(fx, 640.)
            self.assertEqual((fy, cx, cy, w, h), (fx, 640., 360., 1280, 720))

    def test_explicit_independent_calibration_is_used(self):
        with tempfile.TemporaryDirectory() as d:
            session = self.fixture(d, dict(fx=600., fy=605., cx=630., cy=350.,
                                          w=1280, h=720, fov=90.))
            cal = load_calibration(session)
            self.assertEqual(cal['fx'], 600.)
            self.assertEqual(cal['fy'], 605.)

    def test_changed_intrinsics_are_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            session = self.fixture(d)
            meta = json.loads((session / 'meta.json').read_text())
            meta['calibration_constant'] = False
            (session / 'meta.json').write_text(json.dumps(meta), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_calibration(session)

    def test_invalid_explicit_intrinsics_are_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            session = self.fixture(d, dict(fx=0., fy=640., cx=640., cy=360.,
                                          w=1280, h=720, fov=90.))
            with self.assertRaises(ValueError):
                load_calibration(session)

    def test_vo_saves_native_poses_with_no_truth(self):
        import rgbd_vo
        with tempfile.TemporaryDirectory() as d:
            session = self.fixture(d)
            (session/'frames').mkdir()
            (session/'depth').mkdir()
            rng = np.random.default_rng(7)
            texture = rng.integers(0, 256, (720, 1280), dtype=np.uint8)
            texture = cv2.GaussianBlur(texture, (3, 3), 0)
            for i in range(3):
                cv2.imwrite(str(session/'frames'/f'frame_{i:06d}.png'), np.roll(texture, 2*i, axis=1))
                cv2.imwrite(str(session/'depth'/f'frame_{i:06d}.png'), np.full((720,1280),2000,np.uint16))
            out = session/'result'
            with patch('sys.argv',['rgbd_vo.py',str(session),'--out',str(out),'--odometry-only']), contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(rgbd_vo.main(),0)
            rows=(out/'vo_poses.jsonl').read_text().splitlines()
            self.assertEqual(len(rows),3)
            report=json.loads((out/'vo_native.json').read_text())
            self.assertFalse(report['ground_truth_used'])
            self.assertFalse((out/'vo_eval.json').exists())


if __name__ == '__main__':
    unittest.main()
