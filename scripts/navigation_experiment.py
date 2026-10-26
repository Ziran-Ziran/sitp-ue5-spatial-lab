"""Repeatable local UE oracle experiment: prepare, run, or synthetic acceptance.

Windows UE 5.8 only; Python scoring/tests remain portable. Every UE/build process
uses bounded_job's exclusive resource lock and owned tree. Outputs must be new
directories under this repository's ignored output/. No downloads or UI input.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'ue5/tools'))
from bounded_job import run as bounded_run
from stage_navigation import stage,unlinked
from navigation_build import verify_build,file_hash
from navigation_tasks import build_manifest,validate_plan
from navigation_report import evaluate
from runtime_acceptance import validate_synthetic
from semantic_observation_stream import jsonl,inspect_records
from visibility_acceptance import validate_synthetic_visibility


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def write(path,data):
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(data,stream,indent=2,allow_nan=False)


def create_run_directory(root,directory):
    root,directory=Path(root).absolute(),Path(directory).absolute()
    unlinked(directory)
    relative=directory.relative_to(root)
    if len(relative.parts)<2 or relative.parts[0]!='output' or '..' in relative.parts:
        raise ValueError('experiment output must be a new directory under ignored output/')
    directory.mkdir(parents=True,exist_ok=False)
    return directory


def engine_tools(root):
    root=Path(root).absolute()
    version=read(root/'Engine/Build/Build.version')
    if (version['MajorVersion'],version['MinorVersion'])!=(5,8):
        raise ValueError('native bridge currently targets UE 5.8 only')
    editor=root/'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
    builder=root/'Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.exe'
    if not editor.is_file() or not builder.is_file():
        raise ValueError('requested Windows engine executable/tool is missing')
    return editor,builder,version


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','run','synthetic'))
    parser.add_argument('--engine-root',type=Path,required=True)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--lock',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--experiment-map',help='fresh /Game/SpatialLab/<name>; required for prepare/synthetic')
    parser.add_argument('--tasks',type=Path)
    parser.add_argument('--plans',type=Path)
    parser.add_argument('--record-observations',action='store_true',help='opt-in explicit engine local-visibility supervision stream')
    parser.add_argument('--start-before',help='ISO timestamp with timezone; reject starting a heavy job after this')
    parser.add_argument('--finish-before',help='ISO timestamp with timezone; includes a 20s cleanup reserve')
    args=parser.parse_args()
    if os.name!='nt':
        raise ValueError('this launcher targets Windows UE; offline tools/tests support other platforms')
    editor,builder,version=engine_tools(args.engine_root)
    project=args.project.absolute()
    unlinked(project)
    if not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix()):
        raise ValueError('project must be a local independent scene workspace')
    if args.action!='run' and not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.experiment_map or ''):
        raise ValueError('provide a fresh experiment map under /Game/SpatialLab/')
    if args.action=='prepare' and args.record_observations:
        raise ValueError('observations require run or synthetic Game-world execution')
    manifest=plans=None
    if args.action=='run':
        if not args.tasks or not args.plans:
            raise ValueError('run requires frozen --tasks and --plans')
        manifest,plans=read(args.tasks),read(args.plans)
        validate_plan(manifest,plans)
        verify_build(ROOT,project)
    out=create_run_directory(ROOT,args.out)
    common=['-unattended','-nop4','-nosplash','-NullRHI','-NoSound','-stdout','-FullStdOutLogOutput']
    def job(name,command,timeout=600,inputs=None):
        spec=dict(command=[str(x).replace('\\','/') for x in command],cwd=str(ROOT),lock=str(args.lock.absolute()),
                  timeout_s=timeout,lock_wait_s=60,input_sha256=inputs or {})
        if args.start_before: spec['start_before']=args.start_before
        if args.finish_before: spec['finish_before']=args.finish_before
        path=out/(name+'.job.json')
        write(path,spec)
        code=bounded_run(path)
        if code:
            raise RuntimeError(f'{name} failed or lock busy; see preserved spec/status/log; no retry')
    def python_job(name,script,*argv):
        # UE parses Python script paths as escaped strings; use forward slashes.
        invocation=subprocess.list2cmdline([str(ROOT/'ue5/tools'/script).replace('\\','/'),
                                            *[str(x).replace('\\','/') for x in argv]])
        job(name,[editor,project,'-run=pythonscript','-script='+invocation,*common])
    def game_job(map_path,plan_path,result_path):
        built=verify_build(ROOT,project)
        if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',map_path):
            raise ValueError('invalid experiment map identifier')
        binary=ROOT/built['binary']
        map_file=project.parent/'Content'/(map_path[len('/Game/'):]+'.umap')
        inputs={str(binary):built['binary_sha256'],str(plan_path):file_hash(plan_path),str(map_file):file_hash(map_file)}
        sensor=['-SITPObservations='+str(out/'observations.jsonl')] if args.record_observations else []
        job('game',[editor,project,map_path,'-game','-SITPPlan='+str(plan_path),'-SITPResult='+str(result_path),*sensor,
                    *common,'-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'],inputs=inputs)
        if args.record_observations:
            write(out/'observations-summary.json',inspect_records(jsonl(out/'observations.jsonl'),read(result_path)))
    if args.action in ('prepare','synthetic'):
        job('build',[sys.executable,ROOT/'scripts/navigation_build.py','--project',project,
                     '--builder',builder,'--owned-lock',args.lock.absolute()],1200)
    write(out/'environment.json',dict(engine=version,git_head=subprocess.check_output(
        ['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),native_build=verify_build(ROOT,project)))
    if args.action=='prepare':
        python_job('probe','navigation_probe.py','--out',out/'probe.json','--save-map',args.experiment_map)
        manifest=build_manifest(read(out/'probe.json'))
        write(out/'tasks.json',manifest)
        python_job('planning','plan_navigation.py','--tasks',out/'tasks.json','--out',out/'plans.json')
        validate_plan(manifest,read(out/'plans.json'))
        print('PREPARED: frozen tasks and plans; Game-world execution still required')
    elif args.action=='synthetic':
        sensor_fixture=['--semantic-fixture'] if args.record_observations else []
        python_job('fixture','runtime_fixture.py','--map',args.experiment_map,
                   '--out',out/'fixture.json','--plan',out/'plans.json',*sensor_fixture)
        game_job(args.experiment_map,out/'plans.json',out/'execution.json')
        acceptance=validate_synthetic(read(out/'execution.json'))
        write(out/'acceptance.json',acceptance)
        if not acceptance['passed']:
            raise RuntimeError('synthetic acceptance failed; failures preserved')
        if args.record_observations:
            visibility=validate_synthetic_visibility(read(out/'fixture.json'),jsonl(out/'observations.jsonl'))
            write(out/'visibility-acceptance.json',visibility)
            if not visibility['passed']:
                raise RuntimeError('synthetic visibility controls failed; see preserved stream')
        print('PASSED: synthetic UE Game-world integration; city and visual semantics are separate')
    else:
        # Copy exactly validated definitions into this attempt; never overwrite originals.
        write(out/'tasks.json',manifest)
        write(out/'plans.json',plans)
        game_job(manifest['map'],out/'plans.json',out/'execution.json')
        execution=read(out/'execution.json')
        result=evaluate(manifest,execution)
        result['raw_execution_sha256']=hashlib.sha256((out/'execution.json').read_bytes()).hexdigest()
        write(out/'report.json',result)
        if not result['finished']:
            raise RuntimeError('incomplete run; all missing episodes retained in report')
        print(json.dumps(result['by_policy'],indent=2))


if __name__=='__main__':
    try:
        main()
    except (OSError,ValueError,KeyError,RuntimeError) as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
