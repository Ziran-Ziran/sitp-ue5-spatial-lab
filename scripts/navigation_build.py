"""Internal locked stage/build step and verifiable native build provenance."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from stage_navigation import stage,unlinked


def file_hash(path):
    unlinked(path)
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def source_hashes(root):
    source=Path(root)/'ue5/tools/native/SpatialLab'
    return {p.relative_to(root).as_posix():file_hash(p) for p in sorted(source.rglob('*'))
            if p.is_file() and p.suffix in ('.cpp','.h','.cs')}


def record_build(root,project):
    root,project=Path(root),Path(project)
    binary=project.parent/'Binaries/Win64/UnrealEditor-SpatialLab.dll'
    stage_path=project.parent/'.sitp-navigation-stage.json'
    unlinked(stage_path)
    staged=json.loads(stage_path.read_text())
    sources=source_hashes(root)
    for name,expected in sources.items():
        target='Source/'+Path(name).relative_to('ue5/tools/native').as_posix()
        if staged.get('files',{}).get(target)!=expected or file_hash(project.parent/target)!=expected:
            raise ValueError('staged source differs from the native source snapshot')
    record=dict(schema_version=1,native_source_sha256=sources,
                binary= binary.relative_to(root).as_posix(),binary_sha256=file_hash(binary),
                staged_files=staged.get('files',{}))
    (project.parent/'.sitp-navigation-build.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    return record


def verify_build(root,project):
    root,project=Path(root),Path(project)
    record_path=project.parent/'.sitp-navigation-build.json'
    unlinked(record_path)
    if not record_path.is_file():
        raise ValueError('no verified native build record; run prepare or synthetic to build first')
    record=json.loads(record_path.read_text(encoding='utf-8'))
    if record.get('schema_version')!=1 or source_hashes(root)!=record.get('native_source_sha256'):
        raise ValueError('native source changed since build; rebuild before execution')
    binary=project.parent/'Binaries/Win64/UnrealEditor-SpatialLab.dll'
    if file_hash(binary)!=record.get('binary_sha256') or binary.relative_to(root).as_posix()!=record.get('binary'):
        raise ValueError('native binary differs from verified build')
    for name,expected in record.get('staged_files',{}).items():
        if ('..' in Path(name).parts or Path(name).is_absolute() or Path(name).drive
                or not name.startswith('Source/') or file_hash(project.parent/name)!=expected):
            raise ValueError('staged native source changed since build')
    return record


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--builder',type=Path,required=True)
    parser.add_argument('--owned-lock',type=Path,required=True)
    args=parser.parse_args()
    owner=json.loads(args.owned_lock.read_text(encoding='utf-8'))
    if owner.get('pid')!=os.getppid() or owner.get('owner')!='sitp-bounded-job':
        raise RuntimeError('stage/build must run as the owned child of bounded_job')
    project=args.project.absolute()
    before=source_hashes(ROOT)
    for name in ('Binaries','Intermediate','Saved','DerivedDataCache'):
        unlinked(project.parent/name)
    stage(ROOT,project)
    code=subprocess.call([str(args.builder),project.stem+'Editor','Win64','Development',
                          '-Project='+str(project),'-WaitMutex','-NoHotReloadFromIDE','-MaxParallelActions=2'])
    if code:
        raise SystemExit(code)
    if source_hashes(ROOT)!=before:
        raise RuntimeError('native source changed during build; no provenance record written')
    record_build(ROOT,project)
