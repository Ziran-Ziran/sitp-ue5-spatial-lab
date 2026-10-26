"""Owned visual-marker target tasks: actual RGB-D -> class/position -> route -> feedback."""
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
from online_depth_experiment import verify_controls
from stage_navigation import unlinked
from visual_marker_tasks import build_tasks
from visual_marker_report import evaluate


def control_acceptance(report,record):
    scores={r['run_id']:r for r in report['runs']};raw={r['run_id']:r['execution'] for r in record['runs']}
    checks={}
    for category in ('red_marker','blue_marker'):
        checks['front_'+category]=scores['start_00-'+category]['success']
        checks['back_search_'+category]=(scores['start_01-'+category]['success'] and scores['start_01-'+category]['scans']>0)
    checks['fresh_zero_motion_candidate']=(scores['immediate']['success'] and raw['immediate']['sim_s']==0
        and raw['immediate']['continuous_path_length_cm']==0 and raw['immediate']['capture_count']==1)
    checks['absent_category_retained']=scores['missing']['failure_reason']=='target_not_observed_after_scan'
    checks['occluded_category_retained']=scores['occluded']['failure_reason']=='target_not_observed_after_scan'
    checks['invalid_spawn_rejected']=scores['spawn']['failure_reason']=='spawn_blocked'
    checks['all_actual_evidence_audited']=all(r['failure_reason']!='invalid_execution' for r in scores.values())
    checks['no_engine_path_requests']=all(r['engine_path_requests']==0 for r in raw.values())
    return dict(evidence_kind='synthetic_rgb_marker_depth_target_search_controls',passed=report['finished'] and len(scores)==12 and all(checks.values()),checks=checks,
        interpretation='controlled color-marker vision and depth navigation, not furniture recognition or city generalization')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('engine-root','project','lock','out','geometry-controls'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--experiment-map',required=True);parser.add_argument('--start-before');parser.add_argument('--finish-before');args=parser.parse_args()
    project=args.project.absolute();unlinked(project)
    if (not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix())
            or not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.experiment_map)):
        raise ValueError('independent project copy and new owned marker map required')
    if os.name!='nt':raise ValueError('actual UE launcher currently targets Windows')
    build=verify_build(ROOT,project);geometry_controls=verify_controls(args.geometry_controls,project,build)
    editor,_,version=engine_tools(args.engine_root);out=create_run_directory(ROOT,args.out)
    def job(name,command,inputs,timeout=1200):
        spec=dict(command=[str(v).replace('\\','/') for v in command],cwd=str(ROOT),lock=str(args.lock.absolute()),
            timeout_s=timeout,lock_wait_s=60,input_sha256=inputs)
        if args.start_before:spec['start_before']=args.start_before
        if args.finish_before:spec['finish_before']=args.finish_before
        path=out/(name+'.job.json');write(path,spec)
        if bounded_run(path):raise RuntimeError(name+' failed/lock busy; inspect preserved evidence before another task')
    common=['-unattended','-nop4','-nosplash','-NoSound','-stdout','-FullStdOutLogOutput']
    script=ROOT/'ue5/tools/visual_marker_fixture.py'
    argv=[str(script).replace('\\','/'),'--map',args.experiment_map,'--out',str(out/'fixture.json').replace('\\','/'),
        '--truth',str(out/'truth.json').replace('\\','/')]
    job('fixture',[editor,project,'-run=pythonscript','-script='+subprocess.list2cmdline(argv),*common,'-NullRHI'],
        {str(ROOT/build['binary']):build['binary_sha256'],str(script):file_hash(script)})
    map_file=project.parent/'Content'/(args.experiment_map[len('/Game/'):]+'.umap')
    tasks=build_tasks(args.experiment_map,file_hash(map_file));write(out/'tasks.json',tasks)
    import cv2
    import numpy as np
    write(out/'environment.json',dict(engine=version,native_build=build,geometry_controls=geometry_controls,
        python=sys.version,python_executable=sys.executable,numerical_dependencies=dict(numpy=np.__version__,opencv=cv2.__version__),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_tree_dirty=bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True)),
        truth_scope='separate post-run evaluator only; no truth/target coordinate argument supplied to UE or worker'))
    inputs={str(ROOT/build['binary']):build['binary_sha256'],str(map_file):tasks['map_sha256'],str(out/'tasks.json'):file_hash(out/'tasks.json'),
        str(out/'truth.json'):file_hash(out/'truth.json')}
    inputs.update({str(p):file_hash(p) for p in sorted((ROOT/'ue5/tools').glob('*.py'))})
    inputs[str(Path(__file__).absolute())]=file_hash(Path(__file__).absolute())
    for name in ('game.job.json','game.job.status.json'):
        path=args.geometry_controls.absolute()/name;inputs[str(path)]=file_hash(path)
    job('game',[editor,project,tasks['map'],'-game','-SITPVisualMarker='+str(out/'tasks.json'),'-SITPResult='+str(out/'execution.json'),
        '-SITPOnlineOutput='+str(out/'capture'),'-SITPTools='+str(ROOT/'ue5/tools'),'-SITPPython='+sys.executable,*common,
        '-RenderOffscreen','-AllowCommandletRendering','-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'],inputs)
    raw_hash=file_hash(out/'execution.json');truth_hash=file_hash(out/'truth.json');record=read(out/'execution.json')
    report=evaluate(tasks,record,out/'capture',read(out/'truth.json'))
    if file_hash(out/'execution.json')!=raw_hash or file_hash(out/'truth.json')!=truth_hash:raise ValueError('native or independent truth changed during scoring')
    report.update(raw_execution_sha256=raw_hash,independent_truth_sha256=truth_hash);write(out/'report.json',report)
    acceptance=control_acceptance(report,record);write(out/'acceptance.json',acceptance)
    if not acceptance['passed']:raise RuntimeError('controlled marker target validation failed; all raw results retained')
    print(json.dumps(report['overall'],indent=2))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError,RuntimeError) as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
