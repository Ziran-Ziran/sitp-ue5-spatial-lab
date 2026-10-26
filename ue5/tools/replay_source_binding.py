"""Bind replay to the original completed search job, attempt and map bytes."""
import hashlib
import json
import re
from pathlib import Path
from semantic_observation_stream import jsonl,inspect_records


def sha256(path):
    with Path(path).open('rb') as source:return hashlib.file_digest(source,'sha256').hexdigest()


def binding(job_path,observation_path,*,project=None):
    job_path=Path(job_path).absolute();observation_path=Path(observation_path).absolute()
    spec=json.loads(job_path.read_text(encoding='utf-8-sig'))
    status=json.loads(job_path.with_suffix('.status.json').read_text(encoding='utf-8-sig'))
    command=spec['command']
    if (status.get('command')!=command or status.get('exit_code')!=0 or status.get('timed_out')
            or not status.get('tree_cleaned') or not status.get('inputs_verified')
            or status.get('spec_sha256')!=sha256(job_path)
            or status.get('verified_input_sha256')!=spec.get('input_sha256',{})):
        raise ValueError('source search requires a complete matching bounded job')
    def argument(prefix):
        values=[part[len(prefix):] for part in command if part.startswith(prefix)]
        if len(values)!=1:raise ValueError('missing or ambiguous source job argument')
        return values[0]
    if Path(argument('-SITPObservations=')).absolute()!=observation_path:
        raise ValueError('source observation file was not produced by this job')
    argument('-SITPSearch=')
    source_project=Path(command[1]).absolute()
    if source_project.suffix!='.uproject' or (project is not None and source_project!=Path(project).absolute()):
        raise ValueError('source job uses a different scene project')
    execution_path=Path(argument('-SITPResult='))
    execution=json.loads(execution_path.read_text(encoding='utf-8-sig'))
    if execution.get('evidence_kind')!='ue_game_world_observation_dependent_search':
        raise ValueError('source job is not actual observed search execution')
    summary=inspect_records(jsonl(observation_path),execution)
    if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',summary['map']):raise ValueError('invalid source experiment map')
    map_path=source_project.parent/'Content'/(summary['map'][len('/Game/'):]+'.umap')
    hashes={Path(path).absolute():value for path,value in spec.get('input_sha256',{}).items()}
    if hashes.get(map_path)!=sha256(map_path):raise ValueError('source map bytes changed or were not frozen by source job')
    return dict(map_sha256=hashes[map_path],job_sha256=sha256(job_path),execution_sha256=sha256(execution_path))
