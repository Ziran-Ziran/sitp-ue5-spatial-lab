"""Bounded fixed-pitch observations and depth controls in owned local UE maps."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from bounded_job import run as bounded_run
from navigation_experiment import create_run_directory,engine_tools,read,write
from navigation_build import verify_build,file_hash
from stage_navigation import unlinked
from pitch_sweep_protocol import build_sweep
from rendered_replay import validate_plan,build_plan
from semantic_observation_stream import jsonl
from replay_source_binding import binding
from pitch_sweep_capture_report import inspect_sweep_capture
from pitch_sweep_controls_report import inspect_sweep_controls

SOURCES=('pitch_sweep_capture.py','pitch_sweep_protocol.py','rendered_replay.py','semantic_memory.py',
         'semantic_observation_stream.py','replay_source_binding.py')


def verify_controls(directory,spec_path,build):
    directory,spec_path=Path(directory).absolute(),Path(spec_path).absolute()
    unlinked(directory);unlinked(spec_path)
    spec=read(spec_path);status=read(spec_path.with_suffix('.status.json'))
    if (status.get('exit_code')!=0 or status.get('timed_out') or not status.get('tree_cleaned')
            or not status.get('inputs_verified') or status.get('command')!=spec.get('command')
            or status.get('spec_sha256')!=file_hash(spec_path)
            or status.get('verified_input_sha256')!=spec.get('input_sha256',{})):
        raise ValueError('pitch controls need original complete bounded spec/input evidence')
    raw=read(directory/'controls.json');prefix=raw['sequences'][0]['map'].removesuffix('_flat')
    if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',prefix):raise ValueError('invalid owned control map prefix')
    if [row['map'] for row in raw['sequences']]!=[prefix+'_'+case for case in ('flat','wall','drop')]:raise ValueError('control maps mismatch')
    script=ROOT/'ue5/tools/pitch_sweep_controls.py'
    expected='-script='+subprocess.list2cmdline([str(script).replace('\\','/'),'--map-prefix',prefix,'--out',str(directory).replace('\\','/')])
    if spec['command'].count(expected)!=1:raise ValueError('controls do not match actual executed output directory')
    hashes={str(ROOT/build['binary']):build['binary_sha256'],**{str(ROOT/'ue5/tools'/name):file_hash(ROOT/'ue5/tools'/name)
        for name in (*SOURCES,'pitch_sweep_controls.py')}}
    saved={Path(path).absolute():sha for path,sha in spec.get('input_sha256',{}).items()}
    if any(saved.get(Path(path).absolute())!=sha for path,sha in hashes.items()):raise ValueError('control binary or imported scripts changed')
    report=inspect_sweep_controls(directory)
    if not report['passed']:raise ValueError('static pitch controls did not pass')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('controls','run'))
    for name in ('engine-root','project','lock','out'):parser.add_argument('--'+name,type=Path,required=True)
    for name in ('base-plan','observations','source-job','controls','controls-job'):parser.add_argument('--'+name,type=Path)
    parser.add_argument('--map-prefix');parser.add_argument('--start-before');parser.add_argument('--finish-before')
    args=parser.parse_args()
    if os.name!='nt':raise ValueError('UE launcher targets Windows; geometry/replay tests are portable')
    editor,builder,version=engine_tools(args.engine_root);project=args.project.absolute();unlinked(project)
    if not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix()):
        raise ValueError('independent local scene workspace required')
    plan=None
    if args.action=='controls':
        if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.map_prefix or ''):raise ValueError('new owned control prefix required')
    else:
        if not all((args.base_plan,args.observations,args.source_job,args.controls,args.controls_job)):
            raise ValueError('run needs base-plan, observations, source-job, controls and controls-job')
        base=validate_plan(read(args.base_plan));provenance=binding(args.source_job,args.observations,project=project)
        if base!=build_plan(jsonl(args.observations),file_hash(args.observations),source_provenance=provenance,
                frames_per_episode=base['frames_per_episode'],exposure_ev100=base['render_settings']['fixed_exposure_ev100']):
            raise ValueError('base plan differs from original source attempt')
        plan=build_sweep(base)
    out=create_run_directory(ROOT,args.out)
    def job(name,command,inputs=None,timeout=600):
        spec=dict(command=[str(value).replace('\\','/') for value in command],cwd=str(ROOT),lock=str(args.lock.absolute()),
            timeout_s=timeout,lock_wait_s=60,input_sha256=inputs or {})
        if args.start_before:spec['start_before']=args.start_before
        if args.finish_before:spec['finish_before']=args.finish_before
        path=out/(name+'.job.json');write(path,spec)
        if bounded_run(path):raise RuntimeError(name+' failed/lock busy; inspect original evidence before a new task')
        return path
    if args.action=='controls':job('build',[sys.executable,ROOT/'scripts/navigation_build.py','--project',project,'--builder',builder,
        '--owned-lock',args.lock.absolute()],timeout=1200)
    build=verify_build(ROOT,project)
    control_report=verify_controls(args.controls,args.controls_job,build) if args.action=='run' else None
    write(out/'environment.json',dict(engine=version,native_build=build,controls=control_report,
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()))
    inputs={str(ROOT/build['binary']):build['binary_sha256'],**{str(ROOT/'ue5/tools'/name):file_hash(ROOT/'ue5/tools'/name) for name in SOURCES}}
    if args.action=='controls':
        script=ROOT/'ue5/tools/pitch_sweep_controls.py';argv=['--map-prefix',args.map_prefix,'--out',out/'capture']
    else:
        script=ROOT/'ue5/tools/pitch_sweep_capture.py';write(out/'sweep-plan.json',plan)
        argv=['--plan',out/'sweep-plan.json','--observations',args.observations.absolute(),'--source-job',args.source_job.absolute(),'--out',out/'capture']
        inputs.update({str(out/'sweep-plan.json'):file_hash(out/'sweep-plan.json'),str(args.observations.absolute()):file_hash(args.observations),
            str(args.source_job.absolute()):file_hash(args.source_job),
            str(project.parent/'Content'/(base['map'][len('/Game/'):]+'.umap')):provenance['map_sha256']})
        execution=next(value[len('-SITPResult='):] for value in read(args.source_job)['command'] if value.startswith('-SITPResult='))
        inputs[execution]=provenance['execution_sha256']
    inputs[str(script)]=file_hash(script)
    command=subprocess.list2cmdline([str(script).replace('\\','/'),*[str(value).replace('\\','/') for value in argv]])
    spec_path=job('render',[editor,project,'-run=pythonscript','-script='+command,'-AllowCommandletRendering','-RenderOffscreen',
        '-unattended','-nop4','-nosplash','-NoSound','-stdout','-FullStdOutLogOutput'],inputs)
    report=verify_controls(out/'capture',spec_path,build) if args.action=='controls' else inspect_sweep_capture(out/'capture')
    write(out/'acceptance.json' if args.action=='controls' else out/'input-report.json',report)
    print(json.dumps({key:value for key,value in report.items() if key in ('passed','checks','source_frames','image_pairs','input_contract_ready')}))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError,RuntimeError) as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
