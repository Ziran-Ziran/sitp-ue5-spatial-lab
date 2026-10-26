import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from historical_worker import historical_bindings


class HistoricalWorkerTests(unittest.TestCase):
    def setUp(self):
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup);self.root=Path(temporary.name)
        self.tools=self.root/'ue5/tools';self.tools.mkdir(parents=True)
        (self.tools/'worker.py').write_bytes(b'import helper\nimport json\n')
        (self.tools/'helper.py').write_bytes(b'def value():\n    return 1\n')
        self.git('init','-q');self.git('add','ue5/tools');self.git('-c','user.name=Test','-c','user.email=test@example.invalid','commit','-qm','fixture')
        self.commit=self.git('rev-parse','HEAD').decode().strip()
        self.recorded={str(p):hashlib.sha256(p.read_bytes().replace(b'\n',b'\r\n')).hexdigest() for p in self.tools.glob('*.py')}

    def git(self,*args):return subprocess.check_output(['git',*args],cwd=self.root,stderr=subprocess.PIPE)

    def test_exact_historical_closure_matches_recorded_crlf_without_current_source_reads(self):
        (self.tools/'helper.py').write_bytes(b'raise RuntimeError("current changed")\n')
        result=historical_bindings(self.root,self.commit,'worker',self.recorded)
        self.assertEqual(set(result['files']),{'worker.py','helper.py'})
        self.assertTrue(all(row['recorded_sha256']==self.recorded[str(self.tools/name)] for name,row in result['files'].items()))

    def test_modified_code_or_missing_guard_is_not_a_newline_normalization(self):
        for changed in (dict(self.recorded,**{str(self.tools/'helper.py'):'a'*64}),{str(self.tools/'worker.py'):self.recorded[str(self.tools/'worker.py')]}):
            with self.assertRaises(ValueError):historical_bindings(self.root,self.commit,'worker',changed)

    def test_branch_alias_and_uncommitted_working_copy_are_not_historical_proof(self):
        for ref in ('HEAD','main','0'*40):
            with self.subTest(ref=ref),self.assertRaises(ValueError):historical_bindings(self.root,ref,'worker',self.recorded)

    def test_missing_committed_local_import_cannot_masquerade_as_external(self):
        self.git('rm','-q','ue5/tools/helper.py')
        self.git('-c','user.name=Test','-c','user.email=test@example.invalid','commit','-qm','missing helper')
        commit=self.git('rev-parse','HEAD').decode().strip()
        (self.tools/'helper.py').write_bytes(b'def value():\n    return 1\n')
        with self.assertRaises(ValueError):historical_bindings(self.root,commit,'worker',self.recorded)
        recorded={key:value for key,value in self.recorded.items() if Path(key).name!='helper.py'}
        with self.assertRaises(ValueError):historical_bindings(self.root,commit,'worker',recorded)

    def test_git_replace_cannot_substitute_bytes_beneath_an_immutable_commit(self):
        original=self.git('rev-parse',self.commit+':ue5/tools/helper.py').decode().strip()
        changed=b'def value():\n    return 999\n';(self.tools/'helper.py').write_bytes(changed)
        replacement=self.git('hash-object','-w','ue5/tools/helper.py').decode().strip()
        self.git('replace',original,replacement)
        recorded=dict(self.recorded,**{str(self.tools/'helper.py'):hashlib.sha256(changed).hexdigest()})
        with self.assertRaises(ValueError):historical_bindings(self.root,self.commit,'worker',recorded)
