import json
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
from unittest.mock import patch
import test_pitch_sweep_capture as fixtures

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from pitch_sweep_protocol import sweep_digest
from rendered_replay import digest
from rendered_input_report import file_hash
from depth_execution_source import capture_job_binding,SOURCES


class DepthExecutionSourceTests(unittest.TestCase):
    def setUp(self):
        source=fixtures.PitchSweepCaptureTests();source.setUp();self.addCleanup(source.doCleanups)
        self.capture=source.root;self.root=self.capture.parent/('job-'+self.capture.name);self.root.mkdir()
        self.addCleanup(lambda:shutil.rmtree(self.root))
        self.project=self.root/'project.uproject';self.project.write_text('{}',encoding='utf-8')
        self.obs=self.root/'observations.jsonl';self.obs.write_text('synthetic source',encoding='utf-8')
        self.map=self.root/'Content/SpatialLab/Fixture.umap';self.map.parent.mkdir(parents=True);self.map.write_bytes(b'fixture map')
        self.execution=self.root/'execution.json';self.execution.write_text('{}',encoding='utf-8')
        self.source_job=self.root/'source.job.json'
        self.source_job.write_text(json.dumps(dict(command=['ue',str(self.project),'-SITPObservations='+str(self.obs),
            '-SITPResult='+str(self.execution)])),encoding='utf-8')
        base=source.plan['base_plan'];base['source_provenance']['job_sha256']=file_hash(self.source_job)
        base['source_provenance'].update(map_sha256=file_hash(self.map),execution_sha256=file_hash(self.execution))
        base['source_stream_sha256']=file_hash(self.obs);base['plan_sha256']=digest(base)
        source.plan['sweep_sha256']=sweep_digest(source.plan)
        (self.capture/'evaluation/sweep-plan.json').write_text(json.dumps(source.plan),encoding='utf-8')
        source.index.update(sweep_sha256=source.plan['sweep_sha256'],source_provenance=base['source_provenance'],source_stream_sha256=base['source_stream_sha256'])
        source.save();self.binding=base['source_provenance']
        self.plan=self.root/'sweep-plan.json';self.plan.write_text(json.dumps(source.plan),encoding='utf-8')
        self.binary=self.root/'renderer.dll';self.binary.write_bytes(b'old render native')
        tools=Path(__file__).resolve().parents[1]/'ue5/tools';script=tools/'pitch_sweep_capture.py'
        argv=[str(script).replace('\\','/'),'--plan',str(self.plan).replace('\\','/'),'--observations',str(self.obs).replace('\\','/'),
              '--source-job',str(self.source_job).replace('\\','/'),'--out',str(self.capture).replace('\\','/')]
        command=['ue',str(self.project),'-run=pythonscript','-AllowCommandletRendering','-RenderOffscreen','-script='+subprocess.list2cmdline(argv)]
        inputs={str(path):file_hash(path) for path in (self.plan,self.source_job,self.obs,self.binary,self.map,self.execution)}
        inputs.update({str(tools/name):file_hash(tools/name) for name in SOURCES})
        self.job=self.root/'render.job.json';self.job.write_text(json.dumps(dict(command=command,input_sha256=inputs)),encoding='utf-8')
        self.job.with_suffix('.status.json').write_text(json.dumps(dict(command=command,exit_code=0,timed_out=False,
            tree_cleaned=True,inputs_verified=True,spec_sha256=file_hash(self.job),verified_input_sha256=inputs)),encoding='utf-8')

    def bind(self,capture=None):
        with patch('depth_execution_source.search_binding',return_value=self.binding):
            return capture_job_binding(capture or self.capture,self.job,project=self.project)

    def test_completed_capture_binds_even_after_separate_native_rebuild(self):
        self.binary.write_bytes(b'new movement native')
        result=self.bind()
        self.assertEqual(result['render_job_sha256'],file_hash(self.job))
        self.assertNotEqual(result['historical_render_binary_sha256'],file_hash(self.binary))

    def test_other_capture_directory_cannot_reuse_render_status(self):
        other=self.root/'other';shutil.copytree(self.capture,other)
        with self.assertRaises(ValueError):self.bind(other)

    def test_rehashed_spec_and_legacy_status_are_rejected(self):
        data=json.loads(self.job.read_text());data['input_sha256']['extra']='0'*64
        self.job.write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaises(ValueError):self.bind()
