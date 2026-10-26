"""Run an explicit local job with an exclusive resource lock and owned process tree.

Input is a local JSON spec: command (argv), cwd, lock, timeout_s (<=3600),
optional start_before and finish_before (ISO timestamps with timezone).
Never retries, steals locks, invokes a shell or kills unrelated processes.
Windows children start suspended inside a kill-on-close Job Object; even an
early parent exit or runner crash cannot leave a GPU child running unguarded.
"""
import ctypes
import hashlib
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def now():
    return datetime.now(timezone.utc).isoformat()


def remaining_timeout(timeout,finish_before,current=None):
    current=current or datetime.now(timezone.utc)
    remaining=(datetime.fromisoformat(finish_before)-current).total_seconds()
    if remaining<=20:
        raise ValueError('finish deadline leaves no process-tree cleanup budget')
    return min(timeout,remaining-20)


class WindowsTree:
    def __init__(self):
        from ctypes import wintypes as w
        self.api = ctypes.WinDLL('kernel32', use_last_error=True)
        self.api.CreateJobObjectW.argtypes = [ctypes.c_void_p, w.LPCWSTR]
        self.api.CreateJobObjectW.restype = w.HANDLE
        self.api.SetInformationJobObject.argtypes = [w.HANDLE, ctypes.c_int, ctypes.c_void_p, w.DWORD]
        self.api.AssignProcessToJobObject.argtypes = [w.HANDLE, w.HANDLE]
        self.api.TerminateJobObject.argtypes = [w.HANDLE, w.UINT]
        self.api.QueryInformationJobObject.argtypes = [w.HANDLE, ctypes.c_int, ctypes.c_void_p, w.DWORD, ctypes.c_void_p]
        self.api.CloseHandle.argtypes = [w.HANDLE]
        class BasicLimits(ctypes.Structure):
            _fields_ = [('process_time', ctypes.c_int64), ('job_time', ctypes.c_int64),
                        ('flags', w.DWORD), ('minimum', ctypes.c_size_t),
                        ('maximum', ctypes.c_size_t), ('active_limit', w.DWORD),
                        ('affinity', ctypes.c_size_t), ('priority', w.DWORD), ('scheduling', w.DWORD)]
        class ExtendedLimits(ctypes.Structure):
            _fields_ = [('basic', BasicLimits), ('io', ctypes.c_uint64 * 6),
                        ('memory', ctypes.c_size_t * 4)]
        self.handle = self.api.CreateJobObjectW(None, None)
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        limits = ExtendedLimits()
        limits.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not self.api.SetInformationJobObject(self.handle, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
            self.api.CloseHandle(self.handle)
            raise ctypes.WinError(ctypes.get_last_error())

    def attach_and_resume(self, child):
        if not self.api.AssignProcessToJobObject(self.handle, int(child._handle)):
            raise ctypes.WinError(ctypes.get_last_error())
        native = ctypes.WinDLL('ntdll')
        native.NtResumeProcess.argtypes = [ctypes.c_void_p]
        native.NtResumeProcess.restype = ctypes.c_long
        if native.NtResumeProcess(int(child._handle)) != 0:
            raise RuntimeError('cannot resume owned suspended child')

    def close(self):
        if not self.handle:
            return
        try:
            if not self.api.TerminateJobObject(self.handle, 1):
                raise ctypes.WinError(ctypes.get_last_error())
            # JOBOBJECT_BASIC_ACCOUNTING_INFORMATION: four LARGE_INTEGER,
            # followed by page faults, total, active and terminated processes.
            info = (ctypes.c_uint32 * 12)()
            until = time.monotonic() + 15
            while True:
                if not self.api.QueryInformationJobObject(self.handle, 1, ctypes.byref(info), ctypes.sizeof(info), None):
                    raise ctypes.WinError(ctypes.get_last_error())
                if info[10] == 0:
                    break
                if time.monotonic() >= until:
                    raise RuntimeError('owned process tree did not drain; lock retained')
                time.sleep(.02)
        finally:
            self.api.CloseHandle(self.handle)
            self.handle = None


def run(spec_path):
    spec_path = Path(spec_path).resolve()
    spec_bytes = spec_path.read_bytes()
    spec = json.loads(spec_bytes.decode('utf-8-sig'))
    timeout = spec['timeout_s']
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or not 0 < timeout <= 3600:
        raise ValueError('timeout_s must be finite, positive and at most 3600')
    command = spec['command']
    if not isinstance(command, list) or not command or not all(isinstance(x, str) and x for x in command):
        raise ValueError('command must be a nonempty argv list')
    for name in ('start_before', 'finish_before'):
        if name in spec:
            limit = datetime.fromisoformat(spec[name])
            if limit.tzinfo is None:
                raise ValueError('deadline must have timezone')
            remaining = (limit - datetime.now(timezone.utc)).total_seconds()
            if remaining <= 0:
                raise ValueError(name + ' deadline reached')
            if name == 'finish_before':
                timeout = min(timeout, max(.001, remaining - 20))
    status_path = spec_path.with_suffix('.status.json')
    log_path = spec_path.with_suffix('.log')
    if status_path.exists() or log_path.exists():
        raise ValueError('existing job evidence; use a new spec name')
    lock_path = Path(spec['lock']).resolve()
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    token = uuid.uuid4().hex
    state = dict(schema_version=1, command=command, cwd=spec['cwd'], runner_pid=os.getpid(),
                 spec_sha256=hashlib.sha256(spec_bytes).hexdigest(),
                 host=__import__('socket').gethostname(), started_at=now(), status='starting',
                 log=str(log_path), timeout_s=timeout, timed_out=False, tree_cleaned=False, token=token)
    wait = spec.get('lock_wait_s',0)
    if isinstance(wait,bool) or not isinstance(wait,(int,float)) or not math.isfinite(wait) or not 0 <= wait <= 60:
        raise ValueError('lock_wait_s must be between 0 and 60 seconds')
    until = time.monotonic() + wait
    while True:
        try:
            lock = lock_path.open('x', encoding='utf-8')
            break
        except FileExistsError:
            if time.monotonic() >= until:
                print('resource lock busy; no command started', file=sys.stderr)
                return 2
            time.sleep(min(.1, max(.001,until-time.monotonic())))
    # A deadline may pass while queued. Recheck after acquiring, before launch.
    for name in ('start_before','finish_before'):
        if name in spec and datetime.fromisoformat(spec[name]) <= datetime.now(timezone.utc):
            lock.close()
            lock_path.unlink()
            raise ValueError(name + ' deadline reached while waiting')
    if 'finish_before' in spec:
        try:
            timeout = remaining_timeout(timeout,spec['finish_before'])
        except ValueError:
            lock.close()
            lock_path.unlink()
            raise
        state['timeout_s'] = timeout
    def save():
        temporary = status_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(state, indent=2), encoding='utf-8')
        temporary.replace(status_path)
    child = None
    tree = None
    started = time.monotonic()
    try:
        with lock:
            json.dump(dict(owner='sitp-bounded-job', token=token, pid=os.getpid(),
                           host=state['host'], started_at=state['started_at'],
                           spec=str(spec_path), status=str(status_path)), lock, indent=2)
            lock.flush()
            os.fsync(lock.fileno())
        save()
        with log_path.open('xb') as log:
            inputs=spec.get('input_sha256',{})
            if not isinstance(inputs,dict):
                raise ValueError('input_sha256 must map explicit files to expected hashes')
            for filename,expected in inputs.items():
                if not isinstance(expected,str) or len(expected)!=64 or any(c not in '0123456789abcdef' for c in expected):
                    raise ValueError('invalid expected input hash')
                with Path(filename).open('rb') as stream:
                    if hashlib.file_digest(stream,'sha256').hexdigest()!=expected:
                        raise ValueError('input changed before locked process launch')
            state['verified_input_sha256']=dict(inputs)
            state['inputs_verified']=True
            if os.name == 'nt':
                tree = WindowsTree()
                child = subprocess.Popen(command, cwd=spec['cwd'], stdout=log, stderr=log,
                                         creationflags=subprocess.CREATE_NO_WINDOW | 0x4)
                tree.attach_and_resume(child)
            else:
                child = subprocess.Popen(command, cwd=spec['cwd'], stdout=log, stderr=log,
                                         start_new_session=True)
            state.update(child_pid=child.pid, status='running')
            save()
            try:
                state['exit_code'] = child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                state.update(timed_out=True, exit_code=-1)
            state['status'] = 'finished'
    except BaseException as error:
        state.update(status='failed', error=repr(error), exit_code=-1)
    finally:
        try:
            if tree:
                tree.close()
            elif child and os.name != 'nt':
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if child:
                if child.poll() is None:
                    child.kill()  # also handles failure before assigning suspended child
                child.wait(timeout=15)
            state['tree_cleaned'] = True
        except BaseException as error:
            state.update(status='cleanup_failed', cleanup_error=repr(error), exit_code=-1)
        state.update(finished_at=now(), elapsed_s=round(time.monotonic() - started, 3))
        save()
        if state['tree_cleaned']:
            try:
                if json.loads(lock_path.read_text(encoding='utf-8')).get('token') == token:
                    lock_path.unlink()
            except (OSError, ValueError):
                state.update(status='lock_changed', exit_code=-1)
                save()
    return 0 if state.get('exit_code') == 0 else 1


if __name__ == '__main__':
    try:
        raise SystemExit(run(sys.argv[1]))
    except (OSError, ValueError, KeyError) as error:
        print(type(error).__name__ + ': ' + str(error), file=sys.stderr)
        raise SystemExit(2)
