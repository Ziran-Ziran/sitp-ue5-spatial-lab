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


class RenderControlProvenanceTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('render_replay_experiment',ROOT/'scripts/render_replay_experiment.py')
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.a=self.root/'a';self.a.mkdir();self.b=self.root/'b';self.b.mkdir()
        for directory in (self.a,self.b):
            (directory/'render.json').write_text(json.dumps(dict(map='/Game/SpatialLab/Controls')),encoding='utf-8')
        binary=self.root/'renderer.dll';binary.write_bytes(b'synthetic-binary')
        self.build=dict(binary=str(binary),binary_sha256=file_hash(binary))
        script=ROOT/'ue5/tools/render_controls.py'
        command=['ue','project','-script='+subprocess.list2cmdline([str(script).replace('\\','/'),
            '--map','/Game/SpatialLab/Controls','--out',str(self.a).replace('\\','/')])]
        self.spec=self.root/'render.job.json'
        self.spec.write_text(json.dumps(dict(command=command,input_sha256={
            str(binary):file_hash(binary),str(script):file_hash(script)})),encoding='utf-8')
        self.spec.with_suffix('.status.json').write_text(json.dumps(dict(exit_code=0,timed_out=False,
            tree_cleaned=True,inputs_verified=True,command=command,spec_sha256=file_hash(self.spec),
            verified_input_sha256=json.loads(self.spec.read_text())['input_sha256'])),encoding='utf-8')

    def verify(self,directory):
        with patch.object(self.module,'read_controls',return_value={}),patch.object(self.module,'validate_controls',return_value=dict(passed=True)):
            return self.module.verify_controls(directory,self.spec,self.build)

    def test_matching_control_directory_and_job_pass(self):
        self.assertTrue(self.verify(self.a)['passed'])

    def test_other_pixels_cannot_be_paired_with_a_valid_job(self):
        with self.assertRaises(ValueError):self.verify(self.b)

    def test_changed_job_command_must_match_saved_execution_status(self):
        data=json.loads(self.spec.read_text());data['command']=['ue','changed']
        self.spec.write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaises(ValueError):self.verify(self.a)

    def test_rehashed_inputs_cannot_reuse_original_status(self):
        data=json.loads(self.spec.read_text());data['input_sha256']['another.bin']='0'*64
        self.spec.write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaises(ValueError):self.verify(self.a)
