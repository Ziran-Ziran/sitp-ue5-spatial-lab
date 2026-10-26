"""Bounded stopped RGB-D sensing, depth planning and direct Game-world feedback."""
import argparse
import ast
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
from depth_path_experiment import completed_job
from depth_execution_source import capture_job_binding
from pitch_sweep_capture_report import inspect_sweep_capture
from online_depth_tasks import build_tasks,build_controls,validate_tasks
from online_depth_report import evaluate


def policy_sources():
    """Local static import closure, including imports inside called functions.

    Post-run scoring can improve without invalidating unchanged physical worker
    evidence. The worker's complete local Python import closure must still match.
    """
    directory=ROOT/'ue5/tools';pending=['online_depth_policy'];found=set()
    while pending:
        name=pending.pop();path=directory/(name+'.py')
        if path in found or not path.is_file():continue
        found.add(path)
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
            names=([entry.name for entry in node.names] if isinstance(node,ast.Import)
                   else [node.module] if isinstance(node,ast.ImportFrom) and node.module else [])
            pending.extend(name.split('.')[0] for name in names)
    return sorted(found)


def prepare(capture,capture_job,project):
    binding=capture_job_binding(capture,capture_job,project=project)
    inspect_sweep_capture(capture)
    sweep=read(capture/'evaluation/sweep-plan.json');base=sweep['base_plan'];frames=[]
    for index,episode in enumerate(base['episodes']):
        for frame in episode['frames']:
            start=list(frame['camera_position_cm']);start[2]-=150
            frames.append(dict(sequence=f'sequence_{index:03d}',source_run_id=episode['run_id'],frame_id=frame['frame_id'],
                source_frame_id=frame['source_frame_id'],t_source_sim_s=frame['t_source_sim_s'],start_cm=start,
                heading_deg=frame['camera_rotation_deg'][1]))
    binding.update(capture_index_sha256=file_hash(capture/'capture-index.json'),sweep_sha256=sweep['sweep_sha256'],
        camera_poses_sha256=file_hash(capture/'evaluation/camera_poses.jsonl'))
    return build_tasks(base['map'],base['source_provenance']['map_sha256'],frames,
        exposure=base['render_settings']['fixed_exposure_ev100'],source_binding=binding)


def control_acceptance(report,record):
    scores={row['run_id']:row for row in report['runs']};raw={row['run_id']:row['execution'] for row in record['runs']}
    checks={}
    for mode in ('initial_sweep_once','receding_sweep'):
        checks['floor_'+mode]=scores['floor-'+mode]['success']
        checks['spawn_'+mode]=scores['spawn-'+mode]['failure_reason']=='spawn_blocked'
        checks['wall_'+mode]=str(scores['wall-'+mode]['failure_reason']).startswith('geometry_')
        immediate=raw['immediate-'+mode]
        checks['immediate_'+mode]=(scores['immediate-'+mode]['success'] and immediate['capture_count']==0
            and immediate['sim_s']==0 and immediate['continuous_path_length_cm']==0)
    stale=raw['changed-initial_sweep_once'];online=raw['changed-receding_sweep']
    checks['new_obstacle_blocks_stale_route']=(scores['changed-initial_sweep_once']['failure_reason']=='blocked_no_progress'
        and stale['collisions']>0 and stale['capture_count']==1 and stale['intervention_sim_s']>0)
    checks['reobserve_new_obstacle_before_contact']=(online['capture_count']>=2 and online['collisions']==0
        and online['intervention_sim_s']>0 and online['actions'][-1]['t_sim_s']>=online['intervention_sim_s']
        and online['actions'][0]['response']['action']=='move' and online['actions'][-1]['response']['action']=='reject')
    checks['receding_positive_has_multiple_observations']=raw['floor-receding_sweep']['capture_count']>=3
    checks['no_navmesh_requests']=all(r['engine_path_requests']==0 for r in raw.values())
    checks['all_actual_evidence_audited']=all(r['failure_reason']!='invalid_execution' for r in scores.values())
    return dict(evidence_kind='synthetic_online_stopped_depth_and_physics_controls',
        passed=report['finished'] and len(scores)==10 and all(checks.values()),checks=checks)


def verify_controls(directory,project,build):
    directory=Path(directory).absolute();tasks=validate_tasks(read(directory/'tasks.json'))
    if tasks['scope']!='synthetic_online_depth_controls':raise ValueError('independent primitive online controls required')
    game=completed_job(directory/'game.job.json');fixture=completed_job(directory/'fixture.job.json')
    for spec in (game,fixture):
        command=spec['command']
        if (len(command)<3 or Path(command[1]).absolute()!=project.absolute()
                or sum(str(t).lower().endswith('.uproject') for t in command)!=1):raise ValueError('wrong actual control project')
        inputs={Path(p).absolute():sha for p,sha in spec['input_sha256'].items()}
        if inputs.get((ROOT/build['binary']).absolute())!=build['binary_sha256']:raise ValueError('controls must use current native DLL')
    game_inputs={Path(p).absolute():sha for p,sha in game['input_sha256'].items()}
    map_file=project.parent/'Content'/(tasks['map'][len('/Game/'):]+'.umap')
    required={'-game','-SITPOnlineDepth='+str(directory/'tasks.json').replace('\\','/'),
        '-SITPResult='+str(directory/'execution.json').replace('\\','/'),'-SITPOnlineOutput='+str(directory/'capture').replace('\\','/'),
        '-SITPTools='+str(ROOT/'ue5/tools').replace('\\','/'),'-SITPPython='+str(sys.executable).replace('\\','/')}
    for expected in required:
        key=expected.split('=',1)[0].lower()
        if [t for t in game['command'] if t.split('=',1)[0].lower()==key]!=[expected]:
            raise ValueError('unique exact online control parameter required: '+key)
    if (game['command'][2]!=tasks['map'] or not required<=set(game['command'])
            or game_inputs.get(directory/'tasks.json')!=file_hash(directory/'tasks.json')
            or game_inputs.get(map_file.absolute())!=tasks['map_sha256']):raise ValueError('wrong controlled map/tasks/output')
    for path in policy_sources():
        if game_inputs.get(path)!=file_hash(path):raise ValueError('online policy/source changed since controls')
    script=ROOT/'ue5/tools/online_depth_fixture.py'
    expected='-script='+subprocess.list2cmdline([str(script).replace('\\','/'),'--map',tasks['map'],'--out',str(directory/'fixture.json').replace('\\','/')])
    for token in (expected,'-run=pythonscript'):
        key=token.split('=',1)[0].lower()
        if [t for t in fixture['command'] if t.split('=',1)[0].lower()==key]!=[token]:
            raise ValueError('unique exact fixture producer script/run required')
    fixture_inputs={Path(p).absolute():sha for p,sha in fixture['input_sha256'].items()}
    data=read(directory/'fixture.json')
    if (fixture['command'].count(expected)!=1 or fixture_inputs.get(script)!=file_hash(script)
            or data['map']!=tasks['map'] or data['navmesh_build_called'] is not False or data['navigation_data_actors']!=0):
        raise ValueError('wrong no-NavMesh fixture source/output')
    record=read(directory/'execution.json');report=evaluate(tasks,record,directory/'capture');acceptance=control_acceptance(report,record)
    if not acceptance['passed']:raise ValueError('online sensing/controller controls failed')
    return acceptance


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=('prepare','synthetic','run'))
    for name in ('engine-root','project','lock','out'):parser.add_argument('--'+name,type=Path,required=True)
    for name in ('capture','capture-job','tasks','controls'):parser.add_argument('--'+name,type=Path)
    parser.add_argument('--experiment-map');parser.add_argument('--start-before');parser.add_argument('--finish-before');args=parser.parse_args()
    project=args.project.absolute();unlinked(project)
    if not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix()):raise ValueError('independent scene copy required')
    if args.action=='synthetic':
        if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.experiment_map or ''):raise ValueError('fresh owned fixture map required')
    else:
        if not args.capture or not args.capture_job:raise ValueError('original source capture/job required')
        tasks=prepare(args.capture.absolute(),args.capture_job.absolute(),project)
        if args.action=='run' and (not args.tasks or not args.controls or read(args.tasks)!=tasks):raise ValueError('unchanged frozen tasks and controls required')
    out=create_run_directory(ROOT,args.out)
    if args.action=='prepare':write(out/'tasks.json',tasks);print(json.dumps(dict(runs=len(tasks['runs']),scope=tasks['scope'])));return
    if os.name!='nt':raise ValueError('UE launcher currently targets Windows')
    editor,builder,version=engine_tools(args.engine_root)
    def job(name,command,inputs=None,timeout=1200):
        spec=dict(command=[str(v).replace('\\','/') for v in command],cwd=str(ROOT),lock=str(args.lock.absolute()),
            timeout_s=timeout,lock_wait_s=60,input_sha256=inputs or {})
        if args.start_before:spec['start_before']=args.start_before
        if args.finish_before:spec['finish_before']=args.finish_before
        path=out/(name+'.job.json');write(path,spec)
        if bounded_run(path):raise RuntimeError(name+' failed/lock busy; inspect preserved logs before another task')
    if args.action=='synthetic':job('build',[sys.executable,ROOT/'scripts/navigation_build.py','--project',project,'--builder',builder,'--owned-lock',args.lock.absolute()])
    build=verify_build(ROOT,project);common=['-unattended','-nop4','-nosplash','-NoSound','-stdout','-FullStdOutLogOutput']
    if args.action=='synthetic':
        script=ROOT/'ue5/tools/online_depth_fixture.py'
        script_args=[str(script).replace('\\','/'),'--map',args.experiment_map,'--out',str(out/'fixture.json').replace('\\','/')]
        job('fixture',[editor,project,'-run=pythonscript','-script='+subprocess.list2cmdline(script_args),*common,'-NullRHI'],
            {str(ROOT/build['binary']):build['binary_sha256'],str(script):file_hash(script)})
        map_file=project.parent/'Content'/(args.experiment_map[len('/Game/'):]+'.umap');tasks=build_controls(args.experiment_map,file_hash(map_file));controls=None
    else:controls=verify_controls(args.controls,project,build)
    import cv2
    import numpy as np
    write(out/'tasks.json',tasks);write(out/'environment.json',dict(engine=version,native_build=build,controls=controls,
        numerical_dependencies=dict(numpy=np.__version__,opencv=cv2.__version__),
        python=sys.version,python_executable=sys.executable,git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True,cwd=ROOT).strip()))
    map_file=project.parent/'Content'/(tasks['map'][len('/Game/'):]+'.umap')
    inputs={str(ROOT/build['binary']):build['binary_sha256'],str(map_file):tasks['map_sha256'],str(out/'tasks.json'):file_hash(out/'tasks.json')}
    inputs.update({str(p):file_hash(p) for p in sorted((ROOT/'ue5/tools').glob('*.py'))});inputs[str(Path(__file__).absolute())]=file_hash(Path(__file__).absolute())
    job('game',[editor,project,tasks['map'],'-game','-SITPOnlineDepth='+str(out/'tasks.json'),'-SITPResult='+str(out/'execution.json'),
        '-SITPOnlineOutput='+str(out/'capture'),'-SITPTools='+str(ROOT/'ue5/tools'),'-SITPPython='+sys.executable,*common,
        '-RenderOffscreen','-AllowCommandletRendering','-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'],inputs)
    raw_hash=file_hash(out/'execution.json');record=read(out/'execution.json');report=evaluate(tasks,record,out/'capture')
    if file_hash(out/'execution.json')!=raw_hash:raise ValueError('native evidence changed during scoring')
    report['raw_execution_sha256']=raw_hash;write(out/'report.json',report)
    if not report['finished']:raise RuntimeError('incomplete online run; all missing failures retained')
    if args.action=='synthetic':
        acceptance=control_acceptance(report,record);write(out/'acceptance.json',acceptance)
        if not acceptance['passed']:raise RuntimeError('actual online controls failed; inspect raw capture/action feedback')
    print(json.dumps(report['by_mode'],indent=2))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError,RuntimeError) as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
