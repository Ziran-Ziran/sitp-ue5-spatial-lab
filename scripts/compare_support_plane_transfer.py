"""Audit both frozen pixel-derived ground models against actual direct physics."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from navigation_experiment import read
from navigation_build import file_hash,verify_build
from depth_path_experiment import completed_job,verify_controls,unique_argument
from depth_execution_source import capture_job_binding
from depth_execution_report import evaluate
from support_plane_execution import build_bundle,validate_bundle,MODELS
from pitch_sweep_protocol import VIEWS


def paired_summary(bundle,report):
    scores={row['run_id']:row for row in report['runs']};pairs={};groups={};changed=[]
    for row in bundle['runs']:
        key=(row['sequence'],row['frame_id'])
        pairs.setdefault(key,{})[row['mode']]=scores[row['run_id']]
    counts=Counter()
    for key,models in pairs.items():
        before,after=(models[model] for model in MODELS)
        transition=('both_arrived' if before['success'] and after['success'] else
            'plane_only_arrived' if after['success'] else 'horizontal_only_arrived' if before['success'] else 'neither_arrived')
        counts[transition]+=1
        hashes=[bundle['source_files_sha256'][f"{key[0]}/{view['name']}/depth/frame_{key[1]:06d}.png"] for view in VIEWS]
        fingerprint=hashlib.sha256(''.join(hashes).encode()).hexdigest()
        groups.setdefault(fingerprint,[]).append(dict(sequence=key[0],frame_id=key[1],transition=transition))
        if before['success']!=after['success']:
            changed.append(dict(sequence=key[0],frame_id=key[1],transition=transition,depth_input_group_sha256=fingerprint,
                horizontal_status=before['status'],plane_status=after['status']))
    return dict(paired_frames=len(pairs),transitions=dict(counts),changed_frames=changed,
        unique_depth_input_groups=len(groups),changed_unique_depth_input_groups=len({r['depth_input_group_sha256'] for r in changed}),
        depth_input_groups=[dict(depth_input_group_sha256=key,frames=value) for key,value in sorted(groups.items())],
        interpretation='identical three-view depth hashes expose correlated probes; grouping does not change the full fixed denominator or establish independent trials')


def verify_producer(command,project,bundle,directory):
    if (len(command)<3 or Path(command[1]).absolute()!=project or command[2]!=bundle['map']
            or sum(t.lower().endswith('.uproject') for t in command)!=1):raise ValueError('wrong physical transfer project/map')
    for arg in ('-game','-SITPDepthPlan='+str(directory/'plan.json').replace('\\','/'),
            '-SITPResult='+str(directory/'execution.json').replace('\\','/'),
            '-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic'):
        unique_argument(command,arg)


def audit(directory,controls,project,capture,capture_job):
    directory=Path(directory).absolute();project=Path(project).absolute();controls=Path(controls).absolute()
    build=verify_build(ROOT,project);acceptance=verify_controls(controls,project,build)
    bundle=validate_bundle(read(directory/'plan.json'))
    binding=capture_job_binding(capture,capture_job,project=project)
    if bundle!=build_bundle(capture,source_job_binding=binding):raise ValueError('frozen model pair changes original full pixels/poses/source')
    game=completed_job(directory/'game.job.json');command=game['command']
    verify_producer(command,project,bundle,directory)
    hashes={Path(p).absolute():sha for p,sha in game['input_sha256'].items()}
    required={ROOT/build['binary']:build['binary_sha256'],directory/'plan.json':file_hash(directory/'plan.json'),
        project.parent/'Content'/(bundle['map'][len('/Game/'):]+'.umap'):bundle['map_sha256'],
        ROOT/'scripts/depth_path_experiment.py':file_hash(ROOT/'scripts/depth_path_experiment.py')}
    required.update({ROOT/'ue5/tools'/name:sha for name,sha in bundle['geometry_source_sha256'].items()})
    required.update({Path(capture).absolute()/'input'/name:sha for name,sha in bundle['source_files_sha256'].items()})
    for name in ('capture-index.json','evaluation/sweep-plan.json','evaluation/camera_poses.jsonl'):
        path=Path(capture).absolute()/name;required[path]=file_hash(path)
    for path in (Path(capture_job).absolute(),Path(capture_job).absolute().with_suffix('.status.json')):
        required[path]=file_hash(path)
    for name in ('plan.json','game.job.json','game.job.status.json','fixture.job.json','fixture.job.status.json'):
        path=controls/name;required[path]=file_hash(path)
    if any(hashes.get(path)!=sha or file_hash(path)!=sha for path,sha in required.items()):
        raise ValueError('actual transfer input/native/source/control binding changed')
    raw_hash=file_hash(directory/'execution.json')
    report=evaluate(bundle,read(directory/'execution.json'),bundle_validator=validate_bundle)
    if file_hash(directory/'execution.json')!=raw_hash:raise ValueError('raw physical output changed during scoring')
    if not report['finished'] or any(r['failure_reason']=='invalid_execution' for r in report['runs']):
        raise ValueError('incomplete or invalid physical model comparison')
    return dict(schema_version=1,evidence_kind='actual_frozen_depth_ground_model_physical_transfer_comparison',
        native_binary_sha256=build['binary_sha256'],manifest_sha256=bundle['manifest_sha256'],raw_execution_sha256=raw_hash,
        game_job_sha256=file_hash(directory/'game.job.json'),game_status_sha256=file_hash(directory/'game.job.status.json'),
        controls=acceptance,report=report,paired=paired_summary(bundle,report),
        interpretation='all original frozen local probes; engine pose reset/actuation, no live observation, semantic arrival or generalization; sampled model is not a physical safety certificate')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run','controls','project','capture','capture-job','out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();result=audit(args.run,args.controls,args.project,args.capture,args.capture_job)
    with args.out.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result['report']['by_mode'],indent=2))


if __name__=='__main__':main()
