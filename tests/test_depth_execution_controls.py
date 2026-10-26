import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import depth_path_experiment as launcher
from depth_execution_plan import build_controls


class DepthControlProvenanceTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        self.root=Path(temporary.name);self.out=self.root/'controls';self.out.mkdir()
        self.project=self.root/'Scene.uproject';self.project.write_text('{}')
        self.binary=self.root/'native.dll';self.binary.write_bytes(b'native')
        self.script=self.root/'ue5/tools/depth_execution_fixture.py';self.script.parent.mkdir(parents=True)
        self.script.write_text('# fixture')
        self.plan=build_controls('/Game/SpatialLab/Controls','a'*64)
        self.write('plan.json',self.plan);self.write('fixture.json',dict(map=self.plan['map'],navmesh_build_called=False,navigation_data_actors=0))
        self.write('execution.json',{})
        self.build=dict(binary='native.dll',binary_sha256=launcher.file_hash(self.binary))
        normalize=lambda path:str(path).replace('\\','/')
        self.map=self.root/'Content/SpatialLab/Controls.umap'
        self.game=dict(command=['ue',normalize(self.project),self.plan['map'],'-game',
            '-SITPDepthPlan='+normalize(self.out/'plan.json'),'-SITPResult='+normalize(self.out/'execution.json')],
            input_sha256={str(self.binary):self.build['binary_sha256'],str(self.out/'plan.json'):launcher.file_hash(self.out/'plan.json'),str(self.map):'a'*64})
        script_args=[normalize(self.script),'--map',self.plan['map'],'--out',normalize(self.out/'fixture.json')]
        self.fixture=dict(command=['ue',normalize(self.project),'-run=pythonscript','-script='+subprocess.list2cmdline(script_args)],
            input_sha256={str(self.script):launcher.file_hash(self.script),str(self.binary):self.build['binary_sha256']})

    def write(self,name,data):
        (self.out/name).write_text(json.dumps(data),encoding='utf-8')

    def verify(self):
        with patch.object(launcher,'ROOT',self.root),patch.object(launcher,'completed_job',side_effect=[self.game,self.fixture]),\
                patch.object(launcher,'evaluate',return_value={}),patch.object(launcher,'control_acceptance',return_value=dict(passed=True)):
            return launcher.verify_controls(self.out,self.project,self.build)

    def test_completed_control_producers_accept_bound_map_build_and_project(self):
        self.assertTrue(self.verify()['passed'])

    def test_game_without_map_hash_guard_is_not_a_control_proof(self):
        del self.game['input_sha256'][str(self.map)]
        with self.assertRaises(ValueError):self.verify()

    def test_fixture_from_other_project_or_native_build_is_not_a_control_proof(self):
        original=copy.deepcopy(self.fixture)
        self.fixture['command'][1]=str(self.root/'Other.uproject')
        with self.assertRaises(ValueError):self.verify()
        self.fixture=original;self.fixture['input_sha256'][str(self.binary)]='b'*64
        with self.assertRaises(ValueError):self.verify()

    def test_expected_project_or_map_appended_as_unused_token_cannot_mask_other_input(self):
        for producer in ('game','fixture'):
            original=copy.deepcopy(getattr(self,producer))
            command=getattr(self,producer)['command']
            command.append(command[1]);command[1]=str(self.root/'Other.uproject')
            with self.subTest(producer=producer),self.assertRaises(ValueError):self.verify()
            setattr(self,producer,original)
        self.game['command'][2]='/Game/SpatialLab/Other'
        self.game['command'].append(self.plan['map'])
        with self.assertRaises(ValueError):self.verify()

    def test_duplicate_case_insensitive_producer_flags_cannot_hide_actual_inputs(self):
        for producer,flags in (('game',('-game','-SITPDepthPlan=/different/plan.json','-sitpresult=/different/result.json')),
                               ('fixture',('-run=other_commandlet','-SCRIPT=other.py'))):
            original=copy.deepcopy(getattr(self,producer))
            for flag in flags:
                with self.subTest(producer=producer,flag=flag):
                    getattr(self,producer)['command'].insert(3 if producer=='game' else 2,flag)
                    with self.assertRaises(ValueError):self.verify()
                    setattr(self,producer,copy.deepcopy(original))
