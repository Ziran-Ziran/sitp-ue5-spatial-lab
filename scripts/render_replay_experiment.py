"""Bounded RGB-D static-pose controls and replay; no live sensor/model claims."""
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
from rendered_replay import build_plan,validate_plan
from semantic_observation_stream import jsonl
from render_controls_report import read_controls,validate_controls
from rendered_input_report import inspect_capture
from replay_source_binding import binding


def verify_controls(directory,spec_path,build):
    directory,spec_path=Path(directory).absolute(),Path(spec_path).absolute()
    unlinked(directory);unlinked(spec_path)
    spec=read(spec_path);status=read(spec_path.with_suffix('.status.json'))
    if (status.get('exit_code')!=0 or status.get('timed_out') or not status.get('tree_cleaned')
            or not status.get('inputs_verified') or status.get('spec_sha256')!=file_hash(spec_path)
            or status.get('verified_input_sha256')!=spec.get('input_sha256',{})):
        raise ValueError('controls require a complete bounded native render with original spec/input snapshot')
    raw=read(directory/'render.json')
    if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',raw['map']):raise ValueError('invalid controls map')
    expected_script='-script='+subprocess.list2cmdline([str(ROOT/'ue5/tools/render_controls.py').replace('\\','/'),
        '--map',raw['map'],'--out',str(directory).replace('\\','/'),*(['--reuse-fixture'] if raw.get('reused_fixture') else [])])
    if status.get('command')!=spec.get('command') or spec['command'].count(expected_script)!=1:
        raise ValueError('control pixels, output directory and executed bounded job do not match')
    expected={ROOT/build['binary']:build['binary_sha256'],ROOT/'ue5/tools/render_controls.py':file_hash(ROOT/'ue5/tools/render_controls.py')}
    inputs={Path(path).absolute():sha for path,sha in spec.get('input_sha256',{}).items()}
    if any(inputs.get(path)!=sha for path,sha in expected.items()):
        raise ValueError('controls used a different native binary or control script; revalidate current build')
    report=validate_controls(read_controls(directory))
    if not report['passed']:raise ValueError('current control pixels/depth failed acceptance')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('controls','run'))
    parser.add_argument('--engine-root',type=Path,required=True)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--lock',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--experiment-map',help='fresh controls map')
    parser.add_argument('--plan',type=Path)
    parser.add_argument('--observations',type=Path)
    parser.add_argument('--source-job',type=Path)
    parser.add_argument('--reuse-fixture',action='store_true',help='render existing verified primitive fixture without recreating assets')
    parser.add_argument('--controls',type=Path,help='previous validated raw control capture directory')
    parser.add_argument('--controls-job',type=Path,help='bounded spec that produced those controls')
    parser.add_argument('--start-before');parser.add_argument('--finish-before')
    args=parser.parse_args()
    if os.name!='nt':raise ValueError('renderer launcher targets Windows UE; CPU plan/QC tests are portable')
    editor,builder,version=engine_tools(args.engine_root)
    project=args.project.absolute();unlinked(project)
    if not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix()):
        raise ValueError('project must be an independent local scene workspace')
    plan=None
    if args.action=='controls':
        if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.experiment_map or ''):
            raise ValueError('controls require a fresh experiment map')
    else:
        if not all((args.plan,args.observations,args.controls,args.controls_job,args.source_job)):
            raise ValueError('run requires frozen plan, observations, source-job, controls and controls-job')
        plan=validate_plan(read(args.plan))
        source_hash=file_hash(args.observations)
        provenance=binding(args.source_job,args.observations,project=project)
        if plan!=build_plan(jsonl(args.observations),source_hash,source_provenance=provenance,
                           frames_per_episode=plan['frames_per_episode'],exposure_ev100=plan['render_settings']['fixed_exposure_ev100']):
            raise ValueError('frozen plan does not match source attempt and selection')
    out=create_run_directory(ROOT,args.out)
    def job(name,command,timeout=600,inputs=None):
        spec=dict(command=[str(x).replace('\\','/') for x in command],cwd=str(ROOT),lock=str(args.lock.absolute()),
            timeout_s=timeout,lock_wait_s=60,input_sha256=inputs or {})
        if args.start_before:spec['start_before']=args.start_before
        if args.finish_before:spec['finish_before']=args.finish_before
        path=out/(name+'.job.json');write(path,spec)
        if bounded_run(path):raise RuntimeError(name+' failed/lock busy; inspect preserved spec/status/log')
        return path
    if args.action=='controls':
        job('build',[sys.executable,ROOT/'scripts/navigation_build.py','--project',project,
            '--builder',builder,'--owned-lock',args.lock.absolute()],1200)
    build=verify_build(ROOT,project)
    controls=None
    if args.action=='run':controls=verify_controls(args.controls,args.controls_job,build)
    write(out/'environment.json',dict(engine=version,native_build=build,controls=controls,
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()))
    binary=ROOT/build['binary']
    inputs={str(binary):build['binary_sha256']}
    if args.action=='controls':
        script=ROOT/'ue5/tools/render_controls.py'
        argv=['--map',args.experiment_map,'--out',out/'capture']
        if args.reuse_fixture:argv.append('--reuse-fixture')
    else:
        script=ROOT/'ue5/tools/render_replay_capture.py'
        write(out/'plan.json',plan)
        argv=['--plan',out/'plan.json','--observations',args.observations.absolute(),
              '--source-job',args.source_job.absolute(),'--out',out/'capture']
        inputs.update({str(out/'plan.json'):file_hash(out/'plan.json'),str(args.observations.absolute()):source_hash})
        map_file=project.parent/'Content'/(plan['map'][len('/Game/'):]+'.umap')
        inputs[str(map_file)]=plan['source_provenance']['map_sha256']
        inputs[str(args.source_job.absolute())]=plan['source_provenance']['job_sha256']
        source_command=read(args.source_job)['command']
        source_execution=next(part[len('-SITPResult='):] for part in source_command if part.startswith('-SITPResult='))
        inputs[source_execution]=plan['source_provenance']['execution_sha256']
        for name in ('rendered_replay.py','semantic_observation_stream.py','semantic_memory.py','replay_source_binding.py'):
            path=ROOT/'ue5/tools'/name;inputs[str(path)]=file_hash(path)
    inputs[str(script)]=file_hash(script)
    command=subprocess.list2cmdline([str(script).replace('\\','/'),*[str(x).replace('\\','/') for x in argv]])
    spec_path=job('render',[editor,project,'-run=pythonscript','-script='+command,'-AllowCommandletRendering',
        '-RenderOffscreen','-unattended','-nop4','-nosplash','-NoSound','-stdout','-FullStdOutLogOutput'],inputs=inputs)
    if args.action=='controls':
        result=verify_controls(out/'capture',spec_path,build)
        write(out/'acceptance.json',result)
        print('PASSED: asymmetric color, optical depth, pose translation, repeat and empty controls')
    else:
        result=inspect_capture(out/'capture')
        write(out/'input-report.json',result)
        print(json.dumps({key:result[key] for key in ('input_contract_ready','frames','frames_with_valid_depth','frames_with_spatial_rgb_variation')}))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError,RuntimeError) as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
