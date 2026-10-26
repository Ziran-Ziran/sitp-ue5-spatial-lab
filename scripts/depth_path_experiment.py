"""Frozen depth-plan transfer with explicit pose-supervised direct actuation."""
import argparse
from collections import Counter
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
from depth_execution_source import capture_job_binding
from depth_execution_plan import build_bundle,build_controls,validate_bundle,GEOMETRY_FILES
from depth_execution_report import evaluate,control_acceptance


def completed_job(path):
    path=Path(path);spec=read(path);status=read(path.with_suffix('.status.json'))
    if (status.get('exit_code')!=0 or status.get('timed_out') or not status.get('tree_cleaned')
            or status.get('inputs_verified') is not True or status.get('command')!=spec.get('command')
            or status.get('spec_sha256')!=file_hash(path)
            or status.get('verified_input_sha256')!=spec.get('input_sha256',{})):
        raise ValueError('original complete bounded job snapshot required')
    return spec


def verify_controls(directory,project,build):
    directory=Path(directory).absolute();unlinked(directory)
    plan=validate_bundle(read(directory/'plan.json'))
    if plan['execution_scope']!='synthetic_supplied_route_controls':raise ValueError('movement controls must be separate synthetic supplied routes')
    game=completed_job(directory/'game.job.json');fixture=completed_job(directory/'fixture.job.json')
    for producer in (game,fixture):
        argv=producer['command']
        if (len(argv)<3 or Path(argv[1]).absolute()!=project.absolute()
                or sum(str(token).lower().endswith('.uproject') for token in argv)!=1):
            raise ValueError('control producer must execute exactly the expected positional project')
    command=game['command']
    required={str(project).replace('\\','/'),plan['map'],'-game','-SITPDepthPlan='+str(directory/'plan.json').replace('\\','/'),
        '-SITPResult='+str(directory/'execution.json').replace('\\','/')}
    if not required<=set(command) or command[2]!=plan['map']:
        raise ValueError('controls do not match actual executed project/map/plan/output')
    hashes={Path(path).absolute():sha for path,sha in game.get('input_sha256',{}).items()}
    map_file=project.parent/'Content'/(plan['map'][len('/Game/'):]+'.umap')
    if (hashes.get((ROOT/build['binary']).absolute())!=build['binary_sha256']
            or hashes.get(directory/'plan.json')!=file_hash(directory/'plan.json')
            or hashes.get(map_file.absolute())!=plan['map_sha256']):
        raise ValueError('controls used another native build, map or plan')
    script=ROOT/'ue5/tools/depth_execution_fixture.py'
    expected='-script='+subprocess.list2cmdline([str(script).replace('\\','/'),'--map',plan['map'],'--out',str(directory/'fixture.json').replace('\\','/')])
    source_inputs={Path(path).absolute():sha for path,sha in fixture.get('input_sha256',{}).items()}
    if (fixture['command'].count(expected)!=1 or str(project).replace('\\','/') not in fixture['command']
            or '-run=pythonscript' not in fixture['command'] or source_inputs.get(script)!=file_hash(script)
            or source_inputs.get((ROOT/build['binary']).absolute())!=build['binary_sha256']):
        raise ValueError('control fixture source/project/build/output mismatch')
    fixture_record=read(directory/'fixture.json')
    if (fixture_record.get('navmesh_build_called') is not False or fixture_record.get('navigation_data_actors')!=0
            or fixture_record['map']!=plan['map']):
        raise ValueError('no-NavMesh-build primitive controls required')
    report=evaluate(plan,read(directory/'execution.json'));acceptance=control_acceptance(report,read(directory/'execution.json'))
    if not acceptance['passed']:raise ValueError('current native direct-movement controls failed')
    return acceptance


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','synthetic','run'))
    for name in ('engine-root','project','lock','out'):parser.add_argument('--'+name,type=Path,required=True)
    for name in ('capture','capture-job','plan','controls'):parser.add_argument('--'+name,type=Path)
    parser.add_argument('--experiment-map');parser.add_argument('--start-before');parser.add_argument('--finish-before')
    args=parser.parse_args();project=args.project.absolute();unlinked(project)
    if not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix()):
        raise ValueError('independent local scene workspace required')
    if args.action=='synthetic':
        if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.experiment_map or ''):raise ValueError('new controlled experiment map required')
    elif not args.capture or not args.capture_job:raise ValueError('prepare/run requires original capture and capture-job')
    if args.action=='run' and (not args.plan or not args.controls):raise ValueError('run requires frozen plan and native movement controls')
    if args.action!='synthetic':
        binding=capture_job_binding(args.capture,args.capture_job,project=project)
        bundle=build_bundle(args.capture,source_job_binding=binding)
        if args.action=='run' and validate_bundle(read(args.plan))!=bundle:raise ValueError('frozen plan differs from current source pixels/poses/geometry code')
    out=create_run_directory(ROOT,args.out)
    if args.action=='prepare':
        write(out/'plan.json',bundle)
        print(json.dumps(dict(runs=len(bundle['runs']),approved=sum(row['gate_status']=='approved' for row in bundle['runs']),
            note='CPU frozen depth transfer plan only; no physical movement executed')));return
    if os.name!='nt':raise ValueError('UE launcher targets Windows; CPU geometry/reporting is portable')
    editor,builder,version=engine_tools(args.engine_root)
    def job(name,command,inputs=None,timeout=600):
        spec=dict(command=[str(value).replace('\\','/') for value in command],cwd=str(ROOT),lock=str(args.lock.absolute()),
            timeout_s=timeout,lock_wait_s=60,input_sha256=inputs or {})
        if args.start_before:spec['start_before']=args.start_before
        if args.finish_before:spec['finish_before']=args.finish_before
        path=out/(name+'.job.json');write(path,spec)
        if bounded_run(path):raise RuntimeError(name+' failed/lock busy; inspect preserved evidence before a new task')
    common=['-unattended','-nop4','-nosplash','-NullRHI','-NoSound','-stdout','-FullStdOutLogOutput']
    if args.action=='synthetic':job('build',[sys.executable,ROOT/'scripts/navigation_build.py','--project',project,'--builder',builder,
        '--owned-lock',args.lock.absolute()],timeout=1200)
    build=verify_build(ROOT,project);controls=None
    if args.action=='synthetic':
        script=ROOT/'ue5/tools/depth_execution_fixture.py'
        script_args=[str(script).replace('\\','/'),'--map',args.experiment_map,'--out',str(out/'fixture.json').replace('\\','/')]
        job('fixture',[editor,project,'-run=pythonscript','-script='+subprocess.list2cmdline(script_args),*common],
            {str(ROOT/build['binary']):build['binary_sha256'],str(script):file_hash(script)})
        fixture=read(out/'fixture.json');map_file=project.parent/'Content'/(fixture['map'][len('/Game/'):]+'.umap')
        bundle=build_controls(fixture['map'],file_hash(map_file))
    else:controls=verify_controls(args.controls,project,build)
    write(out/'plan.json',bundle);write(out/'environment.json',dict(engine=version,native_build=build,controls=controls,
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()))
    map_file=project.parent/'Content'/(bundle['map'][len('/Game/'):]+'.umap')
    inputs={str(ROOT/build['binary']):build['binary_sha256'],str(map_file):bundle['map_sha256'],str(out/'plan.json'):file_hash(out/'plan.json')}
    if args.action=='run':
        for name,sha in bundle['source_files_sha256'].items():inputs[str(args.capture.absolute()/'input'/name)]=sha
        inputs.update({str(ROOT/'ue5/tools'/name):sha for name,sha in bundle['geometry_source_sha256'].items()})
        for filename in ('capture-index.json','evaluation/sweep-plan.json','evaluation/camera_poses.jsonl'):
            path=args.capture.absolute()/filename;inputs[str(path)]=file_hash(path)
        for path in (args.capture_job.absolute(),args.capture_job.absolute().with_suffix('.status.json')):inputs[str(path)]=file_hash(path)
    job('game',[editor,project,bundle['map'],'-game','-SITPDepthPlan='+str(out/'plan.json'),'-SITPResult='+str(out/'execution.json'),
        *common,'-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'],inputs)
    raw_hash=file_hash(out/'execution.json');raw=read(out/'execution.json');report=evaluate(bundle,raw)
    if file_hash(out/'execution.json')!=raw_hash:raise ValueError('raw execution changed during scoring')
    report['raw_execution_sha256']=raw_hash;write(out/'report.json',report)
    if not report['finished']:raise RuntimeError('incomplete depth transfer; missing cases retained')
    if args.action=='synthetic':
        acceptance=control_acceptance(report,raw);write(out/'acceptance.json',acceptance)
        if not acceptance['passed']:raise RuntimeError('native movement controls failed; inspect raw feedback')
    print(json.dumps(report['by_mode'],indent=2))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError,RuntimeError) as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
