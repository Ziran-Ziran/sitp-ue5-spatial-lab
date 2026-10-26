"""Bounded observation-dependent UE search with explicit engine supervision.

The native runner receives missions and observed frames, never the independent
annotation manifest. All annotations are read for scoring only after execution.
"""
import argparse
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
from navigation_experiment import create_run_directory,engine_tools,read,write
from navigation_build import file_hash,verify_build
from stage_navigation import unlinked
from semantic_search_tasks import build_bundle,export_start_queries,validate_bundle
from semantic_search_report import evaluate
from semantic_observation_stream import inspect_records,jsonl


def synthetic_source(fixture):
    episodes=[]
    anchors={}
    for name,category,start,points in [
        ('visible-chair','chair',[-700,0,10],[[-700,700,10],[700,700,10],[700,0,10]]),
        ('occluded-lamp','lamp',[-700,0,10],[[-700,700,10],[700,700,10],[700,0,10]]),
        ('immediate-chair','chair',[-700,-300,10],[]),
        ('invalid-spawn','chair',[0,0,10],[]),
        ('no-exploration','lamp',[-700,0,10],[])]:
        episodes.append(dict(episode_id=name,category=category,start_cm=start,max_sim_s=20,
            max_path_cm=5000,arrival_tolerance_cm=30,success_radius_cm=300))
        anchors[name]=points
    return (dict(schema_version=1,condition='engine_oracle',map=fixture['map'],
                 entities=fixture['semantic_annotations'],episodes=episodes),anchors)


def synthetic_acceptance(report,execution):
    scores={row['run_id']:row for row in report['runs']}
    checks={}
    for policy in ('frame_only','temporal_memory'):
        for name in ('visible-chair','occluded-lamp','immediate-chair'):
            checks[name+'-'+policy]=scores[name+'-'+policy]['success']
        immediate=next(r for r in execution['runs'] if r['run_id']=='immediate-chair-'+policy)['execution']
        checks['zero-motion-'+policy]=immediate['continuous_path_length_cm']==0 and immediate['sim_s']==0
        checks['invalid-spawn-'+policy]=scores['invalid-spawn-'+policy]['failure_reason']=='spawn_not_on_navmesh'
        checks['no-exploration-'+policy]=scores['no-exploration-'+policy]['failure_reason']=='exploration_exhausted'
        row=next(r for r in execution['runs'] if r['run_id']=='occluded-lamp-'+policy)
        reasons=[item['action'].get('reason') for item in row['execution']['policy_trace']]
        checks['observation-changes-movement-'+policy]=bool(reasons[0]=='explore' and 'observed_goal' in reasons[1:])
    return dict(evidence_kind='synthetic_observation_dependent_search_controls',
                passed=report['finished'] and all(checks.values()),checks=checks,
                interpretation='actual supervised observation-to-movement controls, not RGB recognition or city validation')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare','run','synthetic'))
    parser.add_argument('--engine-root',type=Path,required=True)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--lock',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--tasks',type=Path,help='independent oracle task annotations; prepare only')
    parser.add_argument('--missions',type=Path,help='frozen observed-only missions; run only')
    parser.add_argument('--truth',type=Path,help='posthoc evaluation annotations; never passed to UE')
    parser.add_argument('--experiment-map',help='fresh synthetic map /Game/SpatialLab/<name>')
    parser.add_argument('--start-before')
    parser.add_argument('--finish-before')
    args=parser.parse_args()
    if os.name!='nt':raise ValueError('UE launcher targets Windows; policy and scoring tests are portable')
    editor,builder,version=engine_tools(args.engine_root)
    project=args.project.absolute()
    unlinked(project)
    if not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix()):
        raise ValueError('project must be an independent local scene workspace')
    if args.action=='prepare' and args.tasks is None:raise ValueError('prepare requires fixed --tasks')
    if args.action=='run' and (args.missions is None or args.truth is None):raise ValueError('run requires --missions and separate --truth')
    if args.action=='synthetic' and not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.experiment_map or ''):
        raise ValueError('synthetic requires a fresh experiment map')
    bundle=validate_bundle(read(args.missions)) if args.action=='run' else None
    out=create_run_directory(ROOT,args.out)
    common=['-unattended','-nop4','-nosplash','-NullRHI','-NoSound','-stdout','-FullStdOutLogOutput']
    def job(name,command,timeout=600,inputs=None):
        spec=dict(command=[str(x).replace('\\','/') for x in command],cwd=str(ROOT),lock=str(args.lock.absolute()),
                  timeout_s=timeout,lock_wait_s=60,input_sha256=inputs or {})
        if args.start_before:spec['start_before']=args.start_before
        if args.finish_before:spec['finish_before']=args.finish_before
        path=out/(name+'.job.json');write(path,spec)
        if bounded_run(path):raise RuntimeError(name+' failed/lock busy; inspect preserved spec/status/log before retry')
    def python_job(name,script,*argv):
        command=subprocess.list2cmdline([str(ROOT/'ue5/tools'/script).replace('\\','/'),
            *[str(value).replace('\\','/') for value in argv]])
        job(name,[editor,project,'-run=pythonscript','-script='+command,*common])
    if args.action!='run':
        job('build',[sys.executable,ROOT/'scripts/navigation_build.py','--project',project,
            '--builder',builder,'--owned-lock',args.lock.absolute()],1200)
    build=verify_build(ROOT,project)
    sources=[ROOT/'ue5/tools'/name for name in ('semantic_search_policy.py','semantic_memory.py')]
    policy_hashes={str(path):file_hash(path) for path in sources}
    write(out/'environment.json',dict(engine=version,native_build=build,policy_source_sha256=policy_hashes,
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()))
    truth=None
    if args.action=='prepare':
        source=read(args.tasks)
        write(out/'starts.json',export_start_queries(source))
        python_job('anchors','search_anchors.py','--starts',out/'starts.json','--out',out/'anchors.json')
        anchors=read(out/'anchors.json')
        if anchors['map']!=source['map']:raise ValueError('anchor map mismatch')
        bundle=build_bundle(source,anchors['points_by_episode'])
        write(out/'missions.json',bundle)
        print('PREPARED: observed-only missions; no semantic search has run')
        return
    if args.action=='synthetic':
        python_job('fixture','runtime_fixture.py','--map',args.experiment_map,'--out',out/'fixture.json','--semantic-fixture')
        truth,anchors=synthetic_source(read(out/'fixture.json'))
        bundle=build_bundle(truth,anchors)
        write(out/'truth.json',truth)
    write(out/'missions.json',bundle)
    map_path=bundle['map']
    if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',map_path):raise ValueError('invalid frozen map')
    map_file=project.parent/'Content'/(map_path[len('/Game/'):]+'.umap')
    binary=ROOT/build['binary']
    inputs={str(binary):build['binary_sha256'],str(map_file):file_hash(map_file),
            str(out/'missions.json'):file_hash(out/'missions.json'),**policy_hashes}
    job('game',[editor,project,map_path,'-game','-SITPSearch='+str(out/'missions.json'),
        '-SITPResult='+str(out/'execution.json'),'-SITPObservations='+str(out/'observations.jsonl'),
        '-SITPPolicyTools='+str(ROOT/'ue5/tools'),*common,'-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'],inputs=inputs)
    execution=read(out/'execution.json')
    write(out/'observations-summary.json',inspect_records(jsonl(out/'observations.jsonl'),execution))
    # This is the first read of city annotations in the run path. No truth path
    # or annotation JSON is part of the engine process's command/input contract.
    if truth is None:truth=read(args.truth)
    report=evaluate(bundle,truth,execution,jsonl(out/'observations.jsonl'))
    report['raw_execution_sha256']=file_hash(out/'execution.json')
    write(out/'report.json',report)
    if not report['finished']:raise RuntimeError('incomplete search; missing attempts retained in report')
    if args.action=='synthetic':
        acceptance=synthetic_acceptance(report,execution)
        write(out/'acceptance.json',acceptance)
        if not acceptance['passed']:raise RuntimeError('synthetic controls failed; preserved raw evidence')
    print(json.dumps(report['by_policy'],indent=2))


if __name__=='__main__':
    try:main()
    except (OSError,ValueError,KeyError,RuntimeError) as error:
        print(type(error).__name__+': '+str(error),file=sys.stderr)
        raise SystemExit(1)
