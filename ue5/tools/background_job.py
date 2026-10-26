"""Run one explicit job specification and persist its exit status for resuming.

No scheduler, implicit retry or shell string construction lives here. The caller
starts this helper hidden and records its PID. Each spec/log/status stays inside
the selected experiment directory, and only this helper's child tree is timed out.
"""
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def run(spec_path):
    spec_path = Path(spec_path).resolve()
    spec = json.loads(spec_path.read_text(encoding='utf-8-sig'))
    directory = spec_path.parent
    status_path = directory / (spec_path.stem + '.status.json')
    log_path = directory / (spec_path.stem + '.log')
    if status_path.exists():
        raise RuntimeError('existing job status; use a new explicit job name')
    state = dict(command=spec['command'], cwd=spec['cwd'], runner_pid=os.getpid(),
                 started_at=now(), spec=str(spec_path), log=str(log_path), status='starting')
    def save():
        temporary = status_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(state, indent=2), encoding='utf-8')
        temporary.replace(status_path)
    save()
    started = time.monotonic()
    with log_path.open('wb') as log:
        try:
            flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
            startup = None
            if os.name == 'nt':
                startup = subprocess.STARTUPINFO()
                startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
                startup.wShowWindow = subprocess.SW_HIDE
            child = subprocess.Popen(spec['command'], cwd=spec['cwd'], stdout=log,
                                     stderr=subprocess.STDOUT, creationflags=flags,
                                     startupinfo=startup)
            state.update(child_pid=child.pid, status='running')
            save()
            try:
                exit_code = child.wait(timeout=spec.get('timeout_s', 2700))
            except subprocess.TimeoutExpired:
                if os.name == 'nt' and child.poll() is None:
                    subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'],
                                   stdout=log, stderr=log, check=False,
                                   creationflags=flags)
                elif child.poll() is None:
                    child.kill()
                child.wait(timeout=30)
                state['timed_out'] = True
                exit_code = -1
            state.update(exit_code=exit_code, status='finished')
        except Exception as error:
            state.update(status='failed', error=repr(error), exit_code=-1)
        state.update(finished_at=now(), elapsed_s=round(time.monotonic()-started, 2))
        save()
    return 0 if state.get('exit_code') == 0 else 1


if __name__ == '__main__':
    raise SystemExit(run(sys.argv[1]))
