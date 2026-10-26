"""Read-only historical worker proof against the original locked byte hashes.

Only exact committed text or its LF/CRLF encoding may match; code is never
executed, restored over the working tree, or accepted through a branch alias.
"""
import ast
import hashlib
from pathlib import Path
import re
import subprocess
import sys


def historical_bindings(root,commit,entry,recorded):
    root=Path(root).absolute()
    def git(*args):
        result=subprocess.run(['git','--no-replace-objects',*args],cwd=root,capture_output=True)
        if result.returncode:raise ValueError('historical committed worker object unavailable')
        return result.stdout
    if not isinstance(commit,str) or not re.fullmatch('[0-9a-f]{40}',commit):raise ValueError('full immutable historical commit SHA required')
    if git('cat-file','-t',commit).strip()!=b'commit':raise ValueError('historical source must be a commit')
    git('merge-base','--is-ancestor',commit,'HEAD')
    hashes={Path(path).absolute():sha for path,sha in recorded.items()};pending=[entry];found={}
    while pending:
        name=pending.pop()
        if name in found:continue
        if not isinstance(name,str) or not re.fullmatch('[A-Za-z_][A-Za-z0-9_]{0,127}',name):raise ValueError('bounded local module identifier required')
        relative='ue5/tools/'+name+'.py';spec=commit+':'+relative
        exists=subprocess.run(['git','--no-replace-objects','cat-file','-e',spec],cwd=root,capture_output=True).returncode==0
        if not exists:
            if name==entry:raise ValueError('historical entry module missing')
            package=root/'ue5/tools'/name/'__init__.py'
            if root/relative in hashes or package in hashes or name not in sys.stdlib_module_names|{'cv2','numpy'}:
                raise ValueError('historical local import absent from commit; cannot assume external dependency')
            continue
        if len(found)>=100:raise ValueError('bounded historical static import closure required')
        raw=git('cat-file','blob',spec)
        if len(raw)>1024*1024 or b'\0' in raw:raise ValueError('bounded UTF-8 worker source required')
        canonical=raw.replace(b'\r\n',b'\n');variants=[('git_blob',raw),('lf',canonical),('crlf',canonical.replace(b'\n',b'\r\n'))]
        path=root/relative;expected=hashes.get(path)
        encoding=next((label for label,data in variants if hashlib.sha256(data).hexdigest()==expected),None)
        if encoding is None:raise ValueError('historical code differs from original locked worker byte hash')
        found[name]=dict(git_blob_sha1=git('rev-parse',spec).decode().strip(),recorded_sha256=expected,matching_encoding=encoding)
        for node in ast.walk(ast.parse(raw.decode('utf-8'))):
            names=([alias.name for alias in node.names] if isinstance(node,ast.Import)
                else [node.module] if isinstance(node,ast.ImportFrom) and node.module else [])
            pending.extend(module.split('.')[0] for module in names)
    return dict(source_commit=commit,entry=entry,files={name+'.py':found[name] for name in sorted(found)},
        interpretation='original guarded worker bytes match this immutable committed static import closure; only LF/CRLF encoding normalization allowed; current scoring still replays every saved action')
