"""Bounded rendered-ramp/roof controls and independent frozen-path audit."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from bounded_job import run as bounded_run
from navigation_build import verify_build,file_hash
from navigation_experiment import create_run_directory,engine_tools,read,write
from stage_navigation import unlinked
from depth_path_experiment import completed_job,unique_argument,verify_controls
from depth_execution_report import evaluate
from online_depth_experiment import policy_sources
from support_physics_cases import cases,floor_transform,ceiling_transform
from support_physics_plan import read_plans,build_from_plans,validate_bundle,MODEL


def norm(path):return str(path).replace('\\','/')


def frozen_inputs(spec,required=None):
    hashes={Path(p).absolute():sha for p,sha in spec['input_sha256'].items()}
    if required and any(hashes.get(path.absolute())!=sha for path,sha in required.items()):raise ValueError('original producer omitted required guarded dependency')
    if any(not p.is_file() or file_hash(p)!=sha for p,sha in hashes.items()):raise ValueError('original guarded input changed')
    return hashes


def verify_game_arguments(command,project,bundle,directory):
    if (len(command)<3 or command[1:3]!=[norm(project),bundle['map']]
            or sum(x.lower().endswith('.uproject') for x in command)!=1):raise ValueError('wrong actual physics project/map')
    for arg in ('-game','-SITPDepthPlan='+norm(directory/'tasks.json'),'-SITPResult='+norm(directory/'execution.json'),
            '-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'):unique_argument(command,arg)


def fixture_valid(capture,map_path):
    fixture=read(capture/'fixture.json')
    if (fixture.get('cases')!=cases() or fixture.get('map')!=map_path or fixture.get('finished') is not True
            or fixture.get('navmesh_build_called') is not False or fixture.get('navigation_data_actors')!=0):raise ValueError('fixed no-Nav primitive fixture required')
    expected=[(case,kind,transform) for case in cases() for kind,transform in [('floor',floor_transform(case)),
        *([('ceiling',ceiling_transform(case))] if case['ceiling_vertical_gap_cm'] is not None else [])]]
    if len(fixture['actors'])!=len(expected) or len(fixture['sequences'])!=len(cases()):raise ValueError('all primitive actors/captures required')
    for actor,(case,kind,(center,scale,pitch)) in zip(fixture['actors'],expected):
        if (actor['case_id']!=case['case_id'] or actor['kind']!=kind or actor['collision_profile']!='BlockAll'
                or math.dist(actor['center_cm'],center)>1e-3 or math.dist(actor['scale'],scale)>1e-5 or abs(actor['pitch_deg']-pitch)>1e-4):
            raise ValueError('actual fixture transform/collision changed')
    for index,(sequence,case) in enumerate(zip(fixture['sequences'],cases())):
        if sequence['sequence']!=f'sequence_{index:03d}' or sequence['run_id']!=case['case_id'] or sequence['expected_frames']!=1:
            raise ValueError('fixed complete capture cases required')
        if set(sequence['views'])!={'level','down45','nadir'}:raise ValueError('all three calibrated views required')
        for name,pitch in (('level',0),('down45',45),('nadir',90)):
            rows=sequence['views'][name]
            if len(rows)!=1:raise ValueError('one actual frame per case/view required')
            row=rows[0];expected_position=list(case['start_cm']);expected_position[2]+=150
            if not row['captured'] or math.dist(row['camera_position_cm'],expected_position)>1e-3:
                raise ValueError('capture actual camera differs from fixed source pose')
            from pitch_sweep_protocol import rotation_matches
            if not rotation_matches(row['camera_rotation_deg'],[-pitch,0,0]):raise ValueError('capture pitch changed')
            for key,relative in (('rgb_sha256',row['color_file']),('depth_sha256',row['depth_file'])):
                if file_hash(capture/relative)!=row[key]:raise ValueError('native capture pixels changed')
    return fixture


def reconstruct(directory,project,build,controls):
    directory=Path(directory).absolute();capture=directory/'capture';fixture_job=completed_job(directory/'fixture.job.json')
    binary=(ROOT/build['binary']).absolute()
    fixture_required={p:file_hash(p) for p in policy_sources('support_physics_fixture')}
    fixture_required[binary]=build['binary_sha256']
    inputs=frozen_inputs(fixture_job,fixture_required);command=fixture_job['command']
    if len(command)<3 or Path(command[1]).absolute()!=project or sum(x.lower().endswith('.uproject') for x in command)!=1:
        raise ValueError('wrong fixture project')
    saved=read(directory/'tasks.json') if (directory/'tasks.json').exists() else None
    fixture=read(capture/'fixture.json');map_path=fixture['map']
    if not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',map_path):raise ValueError('owned synthetic map required')
    script=ROOT/'ue5/tools/support_physics_fixture.py'
    unique_argument(command,'-run=pythonscript');unique_argument(command,'-script='+subprocess.list2cmdline([norm(script),'--map',map_path,'--out',norm(capture)]))
    for arg in ('-RenderOffscreen','-AllowCommandletRendering'):unique_argument(command,arg)
    if inputs.get(binary)!=build['binary_sha256'] or inputs.get(script)!=file_hash(script):raise ValueError('current fixture build/source required')
    fixture_valid(capture,map_path);control=verify_controls(controls,project,build)
    map_file=project.parent/'Content'/(map_path[len('/Game/'):]+'.umap')
    plans,pixels=read_plans(capture)
    provenance=dict(kind='synthetic_observed_support_physics',capture_binding={
        'fixture_job_sha256':file_hash(directory/'fixture.job.json'),'fixture_status_sha256':file_hash(directory/'fixture.job.status.json'),
        'fixture_sha256':file_hash(capture/'fixture.json'),'poses_sha256':file_hash(capture/'evaluation/camera_poses.jsonl'),
        'native_binary_sha256':build['binary_sha256'],'controls_game_sha256':file_hash(controls/'game.job.json'),
        'controls_status_sha256':file_hash(controls/'game.job.status.json')})
    bundle=build_from_plans(map_path,file_hash(map_file),plans,source_provenance=provenance,pixel_hashes=pixels)
    if saved is not None and validate_bundle(saved)!=bundle:raise ValueError('saved plan differs from frozen pixels/fixture/source/control')
    return bundle,control


def acceptance(report,record):
    scores={row['run_id']:row for row in report['runs']};raw={row['run_id']:row['execution'] for row in record['runs']}
    checks=dict(all23_complete=report['finished'] and len(scores)==23,
        all_actual_evidence_valid=all(row['failure_reason']!='invalid_execution' for row in scores.values()),
        no_engine_path_requests=all(row['engine_path_requests']==0 for row in raw.values()))
    for case in ('flat','high220'):
        for model in ('horizontal_v1',MODEL):checks[case+'_'+model]=scores[case+'-'+model]['success']
    for case in ('up5','down5'):checks[case+'_plane_arrived']=scores[case+'-'+MODEL]['success']
    checks['high_reference_arrived']=scores['high220-supplied_reference']['success']
    checks['low_reference_blocked_with_contact']=(scores['low170-supplied_reference']['failure_reason']=='blocked_no_progress'
        and bool(scores['low170-supplied_reference']['collisions']))
    checks['sloped_low_reference_blocked_with_contact']=(scores['slope10_low1765-supplied_reference']['failure_reason']=='blocked_no_progress'
        and bool(scores['slope10_low1765-supplied_reference']['collisions']))
    dangerous=[row['run_id'] for row in scores.values() if row['mode']!='supplied_reference'
        and row['gate_status']=='approved' and row['measurement_valid'] and row['collisions']]
    checks['no_approved_model_contact']=not dangerous
    return dict(evidence_kind='synthetic_rendered_support_plane_physics_acceptance',passed=all(checks.values()),checks=checks,
        dangerous_model_approvals=dangerous,
        interpretation='instrumentation controls; model positives/failures and references separately reported, not city navigation')


def audit(directory,project,build,controls):
    directory=Path(directory).absolute();bundle,_=reconstruct(directory,project,build,controls)
    spec=completed_job(directory/'game.job.json');command=spec['command']
    verify_game_arguments(command,project,bundle,directory)
    required={ROOT/build['binary']:build['binary_sha256'],directory/'tasks.json':file_hash(directory/'tasks.json'),
        project.parent/'Content'/(bundle['map'][len('/Game/'):]+'.umap'):bundle['map_sha256'],Path(__file__).absolute():file_hash(Path(__file__))}
    required.update({ROOT/'ue5/tools'/name:sha for name,sha in bundle['geometry_source_sha256'].items()})
    required.update({directory/'capture/input'/name:sha for name,sha in bundle['source_files_sha256'].items()})
    for path in [*(directory/name for name in ('fixture.job.json','fixture.job.status.json')),directory/'capture/fixture.json',
            directory/'capture/evaluation/camera_poses.jsonl',*(controls/name for name in ('game.job.json','game.job.status.json','plan.json','fixture.job.json','fixture.job.status.json','fixture.json'))]:
        required[path]=file_hash(path)
    frozen_inputs(spec,required)
    record=read(directory/'execution.json');report=evaluate(bundle,record,bundle_validator=validate_bundle)
    report.update(raw_execution_sha256=file_hash(directory/'execution.json'),acceptance=acceptance(report,record),
        interpretation='synthetic rendered depth models physically transferred under original height/body/gate budgets; all model failures and separate supplied references retained; not online sensing or semantic/city success')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=('run','audit'))
    for name in ('project','controls','out'):parser.add_argument('--'+name,type=Path,required=True)
    for name in ('engine-root','lock','run'):parser.add_argument('--'+name,type=Path)
    parser.add_argument('--experiment-map');parser.add_argument('--start-before');parser.add_argument('--finish-before');args=parser.parse_args()
    project=args.project.absolute();unlinked(project)
    if not re.fullmatch(r'local/scenes/[a-z0-9-]+/workspace/UE5/[A-Za-z][A-Za-z0-9_]*\.uproject',project.relative_to(ROOT).as_posix()):raise ValueError('independent local project required')
    build=verify_build(ROOT,project);controls=args.controls.absolute();verify_controls(controls,project,build)
    if args.action=='audit':
        if not args.run:raise ValueError('original actual run required')
        report=audit(args.run,project,build,controls);write(args.out,report);print(json.dumps(report['by_mode'],indent=2));return
    if not args.engine_root or not args.lock or not re.fullmatch(r'/Game/SpatialLab/[A-Za-z][A-Za-z0-9_]*',args.experiment_map or ''):
        raise ValueError('current engine, shared lock and new owned map required')
    editor,_,_=engine_tools(args.engine_root);out=create_run_directory(ROOT,args.out)
    def job(name,command,inputs,timeout=300):
        spec=dict(command=[norm(x) for x in command],cwd=str(ROOT),lock=str(args.lock.absolute()),timeout_s=timeout,lock_wait_s=60,input_sha256=inputs)
        if args.start_before:spec['start_before']=args.start_before
        if args.finish_before:spec['finish_before']=args.finish_before
        path=out/(name+'.job.json');write(path,spec)
        if bounded_run(path):raise RuntimeError(name+' failed; inspect preserved evidence')
    common=['-unattended','-nop4','-nosplash','-NoSound','-stdout','-FullStdOutLogOutput']
    inputs={str(p):file_hash(p) for p in sorted((ROOT/'ue5/tools').glob('*.py'))};inputs[str(ROOT/build['binary'])]=build['binary_sha256']
    inputs[str(Path(__file__).absolute())]=file_hash(Path(__file__))
    script=ROOT/'ue5/tools/support_physics_fixture.py';argv=[norm(script),'--map',args.experiment_map,'--out',norm(out/'capture')]
    job('fixture',[editor,project,'-run=pythonscript','-script='+subprocess.list2cmdline(argv),*common,'-RenderOffscreen','-AllowCommandletRendering'],inputs)
    bundle,_=reconstruct(out,project,build,controls);write(out/'tasks.json',bundle)
    game_inputs=dict(inputs);game_inputs[str(out/'tasks.json')]=file_hash(out/'tasks.json')
    game_inputs.update({str(out/'capture/input'/name):sha for name,sha in bundle['source_files_sha256'].items()})
    game_inputs[str(project.parent/'Content'/(args.experiment_map[len('/Game/'):]+'.umap'))]=bundle['map_sha256']
    for path in [*(out/name for name in ('fixture.job.json','fixture.job.status.json')),out/'capture/fixture.json',
            out/'capture/evaluation/camera_poses.jsonl',*(controls/name for name in ('game.job.json','game.job.status.json','plan.json','fixture.job.json','fixture.job.status.json','fixture.json'))]:game_inputs[str(path)]=file_hash(path)
    job('game',[editor,project,bundle['map'],'-game','-SITPDepthPlan='+norm(out/'tasks.json'),'-SITPResult='+norm(out/'execution.json'),
        *common,'-NullRHI','-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'],game_inputs)
    report=audit(out,project,build,controls);write(out/'report.json',report);print(json.dumps(report['by_mode'],indent=2))
    if not report['acceptance']['passed']:raise RuntimeError('physics instrumentation controls failed; all cases retained')


if __name__=='__main__':main()
