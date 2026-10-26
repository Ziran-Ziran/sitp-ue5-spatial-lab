"""Bounded controlled RGB-marker goals in an owned copy of city geometry."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from bounded_job import run as bounded_run
from navigation_experiment import create_run_directory,engine_tools,read,write
from navigation_build import verify_build,file_hash
from depth_path_experiment import completed_job
from online_depth_experiment import prepare as source_tasks,policy_sources
from compare_visual_marker_runs import audit_run as marker_controls,unique_argument
from city_marker_tasks import build_tasks,build_placement_plan,VARIANTS
from visual_marker_report import evaluate
from stage_navigation import unlinked
from historical_worker import historical_bindings
from marker_goal_horizon import POLICIES


def norm(path):return str(path).replace('\\','/')


def source(args):
    return source_tasks(args.capture.absolute(),args.capture_job.absolute(),args.project.absolute())


def fixture_argv(scene,map_path):
    return [norm(ROOT/'ue5/tools/city_marker_fixture.py'),'--plan',norm(scene/'placement-plan.json'),
        '--map',map_path,'--out',norm(scene/'fixture.json'),'--truth',norm(scene/'truth.json')]


def positive_fixture(record,plan):
    placed={r['category'] for r in record['placements'] if r['status']=='placed'}
    if (record['requested_placements']!=2*len(plan['anchors']) or len(record['placements'])!=record['requested_placements']
            or any(not 1<=len(r['queries'])<=4 for r in record['placements'])
            or placed!={'red_marker','blue_marker'} or record['navmesh_build_called'] is not True
            or record['navigation_preparation'].get('populated_tiles',0)<=0):
        raise ValueError('city fixture construction lacks validated color categories; preserve placement failures before navigation')


def inspect_scene(args,build):
    scene=args.scene.absolute();unlinked(scene);descriptor=read(scene/'scene.json');original=source(args)
    if read(scene/'source-tasks.json')!=original or read(scene/'placement-plan.json')!=build_placement_plan(original):
        raise ValueError('changed original source poses/placement plan')
    if set(descriptor)!={'schema_version','map','map_sha256','base_map_sha256','files_sha256'} or descriptor['schema_version']!=1:
        raise ValueError('bound owned city scene descriptor required')
    if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',descriptor['map']) or descriptor['map']==original['map']:
        raise ValueError('separate owned overlay map required')
    expected_files={'source-tasks.json','placement-plan.json','fixture.json','truth.json','fixture.job.json','fixture.job.status.json'}
    if set(descriptor['files_sha256'])!=expected_files or any(file_hash(scene/p)!=sha for p,sha in descriptor['files_sha256'].items()):
        raise ValueError('changed prepared scene files')
    positive_fixture(read(scene/'fixture.json'),read(scene/'placement-plan.json'))
    base_file=args.project.absolute().parent/'Content'/(original['map'][len('/Game/'):]+'.umap')
    map_file=args.project.absolute().parent/'Content'/(descriptor['map'][len('/Game/'):]+'.umap')
    if descriptor['base_map_sha256']!=original['map_sha256'] or file_hash(base_file)!=original['map_sha256'] or file_hash(map_file)!=descriptor['map_sha256']:
        raise ValueError('base city changed or wrong actual overlay')
    producer=completed_job(scene/'fixture.job.json');command=producer['command'];hashes={Path(p).absolute():sha for p,sha in producer['input_sha256'].items()}
    if len(command)<3 or Path(command[1]).absolute()!=args.project.absolute() or sum(t.lower().endswith('.uproject') for t in command)!=1:
        raise ValueError('wrong city fixture project')
    unique_argument(command,'-run=pythonscript');unique_argument(command,'-script='+subprocess.list2cmdline(fixture_argv(scene,descriptor['map'])))
    required={ROOT/build['binary']:build['binary_sha256'],base_file:original['map_sha256'],scene/'placement-plan.json':file_hash(scene/'placement-plan.json')}
    required.update({ROOT/'ue5/tools'/name:file_hash(ROOT/'ue5/tools'/name) for name in ('city_marker_fixture.py','navigation_probe.py')})
    if any(hashes.get(p)!=sha for p,sha in required.items()):raise ValueError('wrong city fixture source/plan/native/base binding')
    binding=dict(base_map_sha256=original['map_sha256'],fixture_plan_sha256=file_hash(scene/'placement-plan.json'),fixture_report_sha256=file_hash(scene/'fixture.json'))
    return descriptor,original,binding,map_file


def audit(args,directory,build,*,worker_source_commit=None):
    directory=Path(directory).absolute();descriptor,original,binding,map_file=inspect_scene(args,build)
    tasks=read(directory/'tasks.json');variant=tasks['planner_variant']
    if tasks!=build_tasks(descriptor['map'],descriptor['map_sha256'],original,binding,variant,goal_horizon_policy=tasks.get('goal_horizon_policy')):raise ValueError('changed/filtered original city marker tasks')
    controls=marker_controls(args.controls,args.project,build,worker_source_commit=worker_source_commit)
    if tasks.get('goal_horizon_policy')!=controls.get('horizon_policy'):raise ValueError('city horizon requires matching current marker controls')
    game=completed_job(directory/'game.job.json');command=game['command'];hashes={Path(p).absolute():sha for p,sha in game['input_sha256'].items()}
    if len(command)<3 or Path(command[1]).absolute()!=args.project.absolute() or command[2]!=tasks['map'] or sum(t.lower().endswith('.uproject') for t in command)!=1:
        raise ValueError('wrong actual city marker project/map')
    for arg in ('-game','-SITPVisualMarker='+norm(directory/'tasks.json'),'-SITPResult='+norm(directory/'execution.json'),
            '-SITPOnlineOutput='+norm(directory/'capture'),'-SITPTools='+norm(ROOT/'ue5/tools'),'-SITPPython='+norm(sys.executable)):
        unique_argument(command,arg)
    required={ROOT/build['binary']:build['binary_sha256'],map_file:tasks['map_sha256'],directory/'tasks.json':file_hash(directory/'tasks.json'),
        args.scene.absolute()/'scene.json':file_hash(args.scene.absolute()/'scene.json'),args.scene.absolute()/'truth.json':file_hash(args.scene.absolute()/'truth.json')}
    historical={};proof=None
    if worker_source_commit is not None:
        proof=historical_bindings(ROOT,worker_source_commit,'visual_marker_depth_policy',game['input_sha256'])
        historical={ROOT/'ue5/tools'/name:row['recorded_sha256'] for name,row in proof['files'].items()}
    required.update(historical or {p:file_hash(p) for p in policy_sources('visual_marker_depth_policy')})
    for name in ('tasks.json','game.job.json','game.job.status.json','fixture.job.json','fixture.job.status.json'):
        path=args.controls.absolute()/name;required[path]=file_hash(path)
    if any(hashes.get(p)!=sha or p not in historical and file_hash(p)!=sha for p,sha in required.items()):raise ValueError('city run changes actual worker/native/map/task/control/scene bindings')
    raw_hash=file_hash(directory/'execution.json');truth_path=args.scene.absolute()/'truth.json';truth_hash=file_hash(truth_path)
    report=evaluate(tasks,read(directory/'execution.json'),directory/'capture',read(truth_path))
    if file_hash(directory/'execution.json')!=raw_hash or file_hash(truth_path)!=truth_hash:raise ValueError('raw city/truth changed during audit')
    if not report['finished'] or any(r['failure_reason']=='invalid_execution' for r in report['runs']):raise ValueError('incomplete or invalid actual city marker evidence')
    report.update(raw_execution_sha256=raw_hash,independent_truth_sha256=truth_hash,scene_sha256=file_hash(args.scene.absolute()/'scene.json'))
    if proof is not None:report['worker_source_binding']=proof
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=('prepare','run','audit','compare'))
    for name in ('project','capture','capture-job','out'):parser.add_argument('--'+name,type=Path,required=True)
    for name in ('engine-root','lock','scene','controls'):parser.add_argument('--'+name,type=Path)
    parser.add_argument('--runs',nargs=2,type=Path);parser.add_argument('--planner-variant',choices=VARIANTS)
    parser.add_argument('--goal-horizon-policy',choices=POLICIES)
    parser.add_argument('--worker-source-commit',help='audit-only immutable historical worker SHA; original guarded bytes must match')
    parser.add_argument('--experiment-map');parser.add_argument('--start-before');parser.add_argument('--finish-before');args=parser.parse_args()
    project=args.project.absolute();unlinked(project)
    if args.worker_source_commit is not None and args.action not in ('audit','compare'):raise ValueError('historical source binding is evaluation-only')
    if not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix()):raise ValueError('independent scene copy required')
    build=verify_build(ROOT,project)
    if args.action in ('audit','compare'):
        if not args.scene or not args.controls or not args.runs:raise ValueError('scene, current controls and actual runs required')
        reports=[audit(args,d,build,worker_source_commit=args.worker_source_commit) for d in args.runs]
        if args.action=='compare':
            if {r['planner_variant'] for r in reports}!=set(VARIANTS) or len({r['scene_sha256'] for r in reports})!=1:
                raise ValueError('both complete variants on the same overlay required')
            if len({r.get('goal_horizon_policy') for r in reports})!=1:raise ValueError('planner comparison must hold goal horizon fixed')
        write(args.out,dict(evidence_kind='actual_city_controlled_color_marker_comparison',reports=reports,
            interpretation='controlled visual color goals in one owned city overlay; all source poses and failures; not furniture recognition/generalization'))
        print(json.dumps([{k:r[k] for k in ('planner_variant','overall')} for r in reports],indent=2));return
    if not args.engine_root or not args.lock:raise ValueError('actual bounded tasks require current engine/lock')
    editor,_,version=engine_tools(args.engine_root);out=create_run_directory(ROOT,args.out)
    def job(name,command,inputs,timeout=1200):
        spec=dict(command=[norm(v) for v in command],cwd=str(ROOT),lock=str(args.lock.absolute()),timeout_s=timeout,lock_wait_s=60,input_sha256=inputs)
        if args.start_before:spec['start_before']=args.start_before
        if args.finish_before:spec['finish_before']=args.finish_before
        path=out/(name+'.job.json');write(path,spec)
        if bounded_run(path):raise RuntimeError(name+' failed/lock busy; inspect preserved evidence before another task')
    common=['-unattended','-nop4','-nosplash','-NoSound','-stdout','-FullStdOutLogOutput']
    if args.action=='prepare':
        if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.experiment_map or ''):raise ValueError('new owned overlay map required')
        original=source(args);plan=build_placement_plan(original);write(out/'source-tasks.json',original);write(out/'placement-plan.json',plan)
        base_file=project.parent/'Content'/(original['map'][len('/Game/'):]+'.umap')
        inputs={str(ROOT/build['binary']):build['binary_sha256'],str(base_file):original['map_sha256'],str(out/'placement-plan.json'):file_hash(out/'placement-plan.json')}
        inputs.update({str(ROOT/'ue5/tools'/n):file_hash(ROOT/'ue5/tools'/n) for n in ('city_marker_fixture.py','navigation_probe.py')})
        job('fixture',[editor,project,'-run=pythonscript','-script='+subprocess.list2cmdline(fixture_argv(out,args.experiment_map)),*common,'-NullRHI'],inputs)
        if file_hash(base_file)!=original['map_sha256']:raise ValueError('base map changed during overlay SaveAs')
        positive_fixture(read(out/'fixture.json'),plan)
        map_file=project.parent/'Content'/(args.experiment_map[len('/Game/'):]+'.umap')
        write(out/'scene.json',dict(schema_version=1,map=args.experiment_map,map_sha256=file_hash(map_file),base_map_sha256=original['map_sha256'],
            files_sha256={n:file_hash(out/n) for n in ('source-tasks.json','placement-plan.json','fixture.json','truth.json','fixture.job.json','fixture.job.status.json')}))
        print(json.dumps({k:read(out/'fixture.json')[k] for k in ('original_actor_count','added_markers','requested_placements')}));return
    if not args.scene or not args.controls or not args.planner_variant:raise ValueError('prepared scene, current marker controls and frozen variant required')
    descriptor,original,binding,map_file=inspect_scene(args,build);controls=marker_controls(args.controls,project,build)
    if args.goal_horizon_policy!=controls.get('horizon_policy'):raise ValueError('matching horizon controls required before city execution')
    tasks=build_tasks(descriptor['map'],descriptor['map_sha256'],original,binding,args.planner_variant,goal_horizon_policy=args.goal_horizon_policy);write(out/'tasks.json',tasks)
    write(out/'environment.json',dict(engine=version,native_build=build,marker_controls=controls['report']['overall'],python=sys.version,
        python_executable=sys.executable,truth_scope='separate post-run scoring only; no beacon coordinates/catalog supplied to native tasks or worker'))
    inputs={str(ROOT/build['binary']):build['binary_sha256'],str(map_file):tasks['map_sha256'],str(out/'tasks.json'):file_hash(out/'tasks.json')}
    inputs.update({str(p):file_hash(p) for p in sorted((ROOT/'ue5/tools').glob('*.py'))})
    for path in (args.scene.absolute()/'scene.json',args.scene.absolute()/'truth.json',Path(__file__).absolute()):inputs[str(path)]=file_hash(path)
    for name in ('tasks.json','game.job.json','game.job.status.json','fixture.job.json','fixture.job.status.json'):
        path=args.controls.absolute()/name;inputs[str(path)]=file_hash(path)
    job('game',[editor,project,tasks['map'],'-game','-SITPVisualMarker='+norm(out/'tasks.json'),'-SITPResult='+norm(out/'execution.json'),
        '-SITPOnlineOutput='+norm(out/'capture'),'-SITPTools='+norm(ROOT/'ue5/tools'),'-SITPPython='+norm(sys.executable),*common,
        '-RenderOffscreen','-AllowCommandletRendering','-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'],inputs)
    report=audit(args,out,build);write(out/'report.json',report);print(json.dumps(report['overall'],indent=2))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError,RuntimeError) as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr);raise SystemExit(1)
