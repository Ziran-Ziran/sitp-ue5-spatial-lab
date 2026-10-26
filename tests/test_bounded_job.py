"""Synthetic subprocesses only: resource ownership and process-tree lifetime."""
import importlib.util
import hashlib
from datetime import datetime,timedelta,timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/bounded_job.py'


class BoundedJobTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.lock = self.root / 'shared.lock'

    def run_job(self, code, timeout=10, wait=0, inputs=None):
        spec = self.root / 'job.json'
        spec.write_text(json.dumps(dict(command=[sys.executable, '-c', code],
                                       cwd=str(self.root), timeout_s=timeout,
                                       lock=str(self.lock), lock_wait_s=wait,input_sha256=inputs or {})), encoding='utf-8')
        result = subprocess.run([sys.executable, str(SCRIPT), str(spec)],
                                capture_output=True, text=True, timeout=20)
        status = self.root / 'job.status.json'
        return result, json.loads(status.read_text()) if status.exists() else None

    def test_success_records_pid_log_exit_and_releases_lock(self):
        result, state = self.run_job("print('fixture-output')")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(state['exit_code'], 0)
        self.assertGreater(state['child_pid'], 0)
        self.assertTrue(state['tree_cleaned'])
        self.assertIn('fixture-output', (self.root / 'job.log').read_text())
        self.assertFalse(self.lock.exists())

    def test_busy_lock_is_preserved_and_command_never_runs(self):
        self.lock.write_text('another owner; do not remove')
        result, state = self.run_job("open('unexpected','w').close()")
        self.assertEqual(result.returncode, 2)
        self.assertFalse((self.root / 'unexpected').exists())
        self.assertEqual(self.lock.read_text(), 'another owner; do not remove')

    def test_status_freezes_original_spec_and_verified_inputs(self):
        source=self.root/'fixture.bin';source.write_bytes(b'original')
        inputs={str(source):hashlib.sha256(source.read_bytes()).hexdigest()}
        result,state=self.run_job("print('snapshot')",inputs=inputs)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(state['spec_sha256'],hashlib.sha256((self.root/'job.json').read_bytes()).hexdigest())
        self.assertEqual(state['verified_input_sha256'],inputs)

    def test_timeout_kills_grandchild_before_releasing_lock(self):
        code = ("import subprocess,sys,time; "
                "subprocess.Popen([sys.executable,'-c',"
                "\"import time; time.sleep(1.5); open('escaped','w').close()\"]); "
                "time.sleep(10)")
        result, state = self.run_job(code, .4)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertTrue(state['timed_out'])
        self.assertTrue(state['tree_cleaned'])
        time.sleep(1.6)
        self.assertFalse((self.root / 'escaped').exists())
        self.assertFalse(self.lock.exists())

    def test_parent_success_cannot_leave_grandchild_running(self):
        code = ("import subprocess,sys; subprocess.Popen([sys.executable,'-c',"
                "\"import time; time.sleep(1); open('escaped','w').close()\"])")
        result, state = self.run_job(code)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(state['tree_cleaned'])
        time.sleep(1.1)
        self.assertFalse((self.root / 'escaped').exists())

    def test_failed_launch_releases_owned_lock(self):
        result, state = self.run_job("raise SystemExit(7)")
        self.assertEqual(result.returncode, 1)
        self.assertEqual(state['exit_code'], 7)
        self.assertFalse(self.lock.exists())

    def test_existing_status_prevents_reusing_a_run(self):
        (self.root / 'job.status.json').write_text('{"previous":true}')
        result, _ = self.run_job("open('unexpected','w').close()")
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / 'unexpected').exists())
        self.assertFalse(self.lock.exists())

    def test_waiting_for_owner_release_starts_command_once(self):
        self.lock.write_text('synthetic-owner')
        timer=threading.Timer(.3,self.lock.unlink)
        timer.start()
        self.addCleanup(timer.join)
        result,state=self.run_job("print('once')",wait=2)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual((self.root/'job.log').read_text().count('once'),1)
        self.assertTrue(state['tree_cleaned'])

    def test_waiting_time_is_deducted_from_the_finish_deadline(self):
        spec=importlib.util.spec_from_file_location('bounded_job',SCRIPT)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        start=datetime(2026,1,1,tzinfo=timezone.utc)
        deadline=(start+timedelta(seconds=70)).isoformat()
        self.assertEqual(module.remaining_timeout(50,deadline,start+timedelta(seconds=40)),10)
        with self.assertRaises(ValueError):
            module.remaining_timeout(50,deadline,start+timedelta(seconds=50))

    def test_changed_binary_or_input_cannot_launch_after_lock_wait(self):
        path=self.root/'binary.fixture'
        path.write_bytes(b'changed fixture')
        result,state=self.run_job("open('unexpected','w').close()",inputs={str(path):'0'*64})
        self.assertEqual(result.returncode,1)
        self.assertFalse((self.root/'unexpected').exists())
        self.assertFalse(self.lock.exists())
        self.assertTrue(state['tree_cleaned'])


if __name__ == '__main__':
    unittest.main()
