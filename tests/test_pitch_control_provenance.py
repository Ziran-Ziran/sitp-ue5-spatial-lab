import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from navigation_build import file_hash


class PitchControlProvenanceTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('pitch_sweep_experiment',ROOT/'scripts/pitch_sweep_experiment.py')
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.directory=self.root/'capture';self.directory.mkdir()
        self.other=self.root/'other';self.other.mkdir()
        raw=dict(sequences=[dict(map='/Game/SpatialLab/Fixture_'+kind) for kind in ('flat','wall','drop')])
        for directory in (self.directory,self.other):(directory/'controls.json').write_text(json.dumps(raw),encoding='utf-8')
        binary=self.root/'renderer.dll';binary.write_bytes(b'synthetic-native')
        self.build=dict(binary=str(binary),binary_sha256=file_hash(binary))
        script=ROOT/'ue5/tools/pitch_sweep_controls.py'
        command=['ue','project','-script='+subprocess.list2cmdline([str(script).replace('\\','/'),
            '--map-prefix','/Game/SpatialLab/Fixture','--out',str(self.directory).replace('\\','/')])]
        inputs={str(binary):file_hash(binary),**{str(ROOT/'ue5/tools'/name):file_hash(ROOT/'ue5/tools'/name)
            for name in (*self.module.SOURCES,'pitch_sweep_controls.py')}}
        self.spec=self.root/'render.job.json';self.spec.write_text(json.dumps(dict(command=command,input_sha256=inputs)),encoding='utf-8')
        self.spec.with_suffix('.status.json').write_text(json.dumps(dict(command=command,exit_code=0,timed_out=False,
            tree_cleaned=True,inputs_verified=True,spec_sha256=file_hash(self.spec),verified_input_sha256=inputs)),encoding='utf-8')

    def verify(self,directory):
        with patch.object(self.module,'inspect_sweep_controls',return_value=dict(passed=True)):
            return self.module.verify_controls(directory,self.spec,self.build)

    def test_original_complete_control_job_matches(self):self.assertTrue(self.verify(self.directory)['passed'])

    def test_other_control_output_cannot_reuse_status(self):
        with self.assertRaises(ValueError):self.verify(self.other)

    def test_rehashed_spec_cannot_reuse_unchanged_completed_status(self):
        data=json.loads(self.spec.read_text());data['input_sha256']['new-input']='0'*64
        self.spec.write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaises(ValueError):self.verify(self.directory)

    def test_different_current_binary_invalidates_previous_controls(self):
        self.build['binary_sha256']='0'*64
        with self.assertRaises(ValueError):self.verify(self.directory)
