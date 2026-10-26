import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import online_depth_experiment as launcher
from online_depth_tasks import build_controls


class OnlineControlProducerTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name);self.out=self.root/'controls';self.out.mkdir()
        self.project=self.root/'Scene.uproject';self.project.write_text('{}')
        self.binary=self.root/'native.dll';self.binary.write_bytes(b'native')
        self.script=self.root/'ue5/tools/online_depth_fixture.py';self.script.parent.mkdir(parents=True);self.script.write_text('#fixture')
        self.worker=self.script.parent/'online_depth_policy.py';self.worker.write_text('import numerical_helper')
        self.helper=self.script.parent/'numerical_helper.py';self.helper.write_text('#worker dependency')
        self.scorer=self.script.parent/'online_depth_report.py';self.scorer.write_text('#post-run evaluator')
        self.tasks=build_controls('/Game/SpatialLab/Controls','a'*64)
        for name,value in [('tasks.json',self.tasks),('execution.json',{}),
                ('fixture.json',dict(map=self.tasks['map'],navmesh_build_called=False,navigation_data_actors=0))]:
            (self.out/name).write_text(json.dumps(value))
        self.build=dict(binary='native.dll',binary_sha256=launcher.file_hash(self.binary))
        norm=lambda p:str(p).replace('\\','/')
        self.game=dict(command=['ue',norm(self.project),self.tasks['map'],'-game',
            '-SITPOnlineDepth='+norm(self.out/'tasks.json'),'-SITPResult='+norm(self.out/'execution.json'),
            '-SITPOnlineOutput='+norm(self.out/'capture'),'-SITPTools='+norm(self.root/'ue5/tools'),'-SITPPython='+norm(sys.executable)],
            input_sha256={str(self.binary):self.build['binary_sha256'],str(self.out/'tasks.json'):launcher.file_hash(self.out/'tasks.json'),
                str(self.root/'Content/SpatialLab/Controls.umap'):'a'*64,str(self.script):launcher.file_hash(self.script)})
        self.game['input_sha256'].update({str(p):launcher.file_hash(p) for p in (self.worker,self.helper,self.scorer)})
        args=[norm(self.script),'--map',self.tasks['map'],'--out',norm(self.out/'fixture.json')]
        self.fixture=dict(command=['ue',norm(self.project),'-run=pythonscript','-script='+subprocess.list2cmdline(args)],
            input_sha256={str(self.binary):self.build['binary_sha256'],str(self.script):launcher.file_hash(self.script)})

    def verify(self):
        with patch.object(launcher,'ROOT',self.root),patch.object(launcher,'completed_job',side_effect=[self.game,self.fixture]),\
                patch.object(launcher,'evaluate',return_value={}),patch.object(launcher,'control_acceptance',return_value={'passed':True}):
            return launcher.verify_controls(self.out,self.project,self.build)

    def test_exact_producer_parameters_accept_current_controls(self):
        self.assertTrue(self.verify()['passed'])

    def test_duplicate_task_and_other_policy_or_python_are_rejected(self):
        original=copy.deepcopy(self.game)
        self.game['command'].insert(4,'-SITPOnlineDepth=/different/tasks.json')
        with self.assertRaises(ValueError):self.verify()
        for prefix in ('-SITPTools=','-SITPPython='):
            self.game=copy.deepcopy(original)
            index=next(i for i,t in enumerate(self.game['command']) if t.startswith(prefix))
            self.game['command'][index]=prefix+'/different/location'
            with self.subTest(prefix=prefix),self.assertRaises(ValueError):self.verify()

    def test_case_variant_duplicate_output_or_fixture_script_is_rejected(self):
        self.game['command'].append('-sitpresult=/different/execution.json')
        with self.assertRaises(ValueError):self.verify()
        self.game['command'].pop();self.fixture['command'].append('-script=/different/fixture.py')
        with self.assertRaises(ValueError):self.verify()

    def test_worker_dependencies_are_guarded_while_independent_scoring_can_improve(self):
        self.scorer.write_text('#improved independent audit')
        self.assertTrue(self.verify()['passed'])
        self.helper.write_text('#changed executed dependency')
        with self.assertRaises(ValueError):self.verify()
