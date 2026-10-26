"""Bind frozen pixels to their completed producer; preserve historical DLL hash."""
import json
from pathlib import Path
import subprocess
from replay_source_binding import binding as search_binding
from pitch_sweep_capture_report import inspect_sweep_capture
from rendered_input_report import file_hash,unlinked

SOURCES=('pitch_sweep_capture.py','pitch_sweep_protocol.py','rendered_replay.py','semantic_memory.py',
         'semantic_observation_stream.py','replay_source_binding.py')


def read(path):return json.loads(path.read_text(encoding='utf-8-sig'))


def capture_job_binding(capture,job_path,*,project=None):
    capture,job_path=Path(capture).absolute(),Path(job_path).absolute();unlinked(capture);unlinked(job_path)
    inspect_sweep_capture(capture)
    plan=read(capture/'evaluation/sweep-plan.json');base=plan['base_plan']
    spec=read(job_path);status_path=job_path.with_suffix('.status.json');status=read(status_path)
    if (status.get('exit_code')!=0 or status.get('timed_out') or not status.get('tree_cleaned')
            or not status.get('inputs_verified') or status.get('command')!=spec.get('command')
            or status.get('spec_sha256')!=file_hash(job_path)
            or status.get('verified_input_sha256')!=spec.get('input_sha256',{})):
        raise ValueError('source capture requires its original completed bounded job snapshot')
    hashes={Path(path).absolute():sha for path,sha in spec['input_sha256'].items()}
    matches=[path for path,sha in hashes.items() if sha==base['source_provenance']['job_sha256']]
    if len(matches)!=1:raise ValueError('capture must freeze the original search job')
    source_job=matches[0];source_command=read(source_job)['command']
    def argument(prefix):
        found=[part[len(prefix):] for part in source_command if part.startswith(prefix)]
        if len(found)!=1:raise ValueError('ambiguous original search output')
        return Path(found[0]).absolute()
    observations=argument('-SITPObservations=');execution=argument('-SITPResult=')
    source_project=Path(source_command[1]).absolute()
    if project is not None and source_project!=Path(project).absolute():raise ValueError('different source scene project')
    if len(spec['command'])<2 or Path(spec['command'][1]).absolute()!=source_project:raise ValueError('render/search project mismatch')
    if search_binding(source_job,observations,project=source_project)!=base['source_provenance']:
        raise ValueError('source search/map/execution changed since capture')
    map_file=source_project.parent/'Content'/(base['map'][len('/Game/'):]+'.umap')
    plan_file=job_path.parent/'sweep-plan.json'
    expected_inputs={source_job:base['source_provenance']['job_sha256'],observations:base['source_stream_sha256'],
        execution:base['source_provenance']['execution_sha256'],map_file:base['source_provenance']['map_sha256']}
    if read(plan_file)!=plan:raise ValueError('executed sweep plan differs from captured evaluation plan')
    expected_inputs[plan_file]=file_hash(plan_file)
    expected_inputs.update({Path(__file__).parent/name:file_hash(Path(__file__).parent/name) for name in SOURCES})
    if any(hashes.get(path)!=sha or file_hash(path)!=sha for path,sha in expected_inputs.items()):
        raise ValueError('capture producer inputs or imported source changed')
    argv=[str(Path(__file__).parent/'pitch_sweep_capture.py').replace('\\','/'),'--plan',str(plan_file).replace('\\','/'),
        '--observations',str(observations).replace('\\','/'),'--source-job',str(source_job).replace('\\','/'),
        '--out',str(capture).replace('\\','/')]
    if spec['command'].count('-script='+subprocess.list2cmdline(argv))!=1 or not {'-run=pythonscript','-AllowCommandletRendering','-RenderOffscreen'}<=set(spec['command']):
        raise ValueError('capture directory differs from actual bounded render command')
    binaries=[sha for path,sha in hashes.items() if path.suffix.lower()=='.dll']
    if len(binaries)!=1:raise ValueError('missing historical native render binary snapshot')
    return dict(render_job_sha256=file_hash(job_path),render_status_sha256=file_hash(status_path),historical_render_binary_sha256=binaries[0])
