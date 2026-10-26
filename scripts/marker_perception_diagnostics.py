"""Audit historical controlled marker trials, then diagnose saved-pixel stages."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from navigation_build import verify_build,file_hash
from navigation_experiment import read
from compare_visual_marker_runs import audit_run as audit_controls
from city_marker_experiment import audit as audit_city
from marker_visibility_report import analyze_record,control_acceptance
from marker_extent_replay import source_snapshot


def evaluate(args):
    directory=args.run.absolute();tasks=read(directory/'tasks.json');build=verify_build(ROOT,args.project.absolute())
    raw_hash=file_hash(directory/'execution.json');sources=source_snapshot('marker_visibility_report')
    source_scripts={name:file_hash(ROOT/'scripts'/name) for name in
        ('marker_perception_diagnostics.py','historical_worker.py','city_marker_experiment.py','compare_visual_marker_runs.py')}
    if tasks['scope']=='synthetic_controlled_visual_marker_search':
        audited=audit_controls(directory,args.project,build,worker_source_commit=args.worker_source_commit)
        native=audited['report'];proof=audited['worker_source_binding'];truth_path=directory/'truth.json'
    elif tasks['scope']=='frozen_city_marker_source_pose_tasks':
        if not all(getattr(args,key,None) for key in ('scene','controls','capture','capture_job')):
            raise ValueError('city diagnostics require original scene, controls, capture and capture-job')
        native=audit_city(args,directory,build,worker_source_commit=args.worker_source_commit)
        proof=native.get('worker_source_binding');truth_path=args.scene.absolute()/'truth.json'
    else:raise ValueError('known controlled-box fixture scope required')
    if file_hash(directory/'execution.json')!=raw_hash:raise ValueError('raw output changed during original execution audit')
    truth_hash=file_hash(truth_path);result=analyze_record(tasks,read(directory/'execution.json'),read(truth_path))
    if (file_hash(directory/'execution.json')!=raw_hash or file_hash(truth_path)!=truth_hash
            or source_snapshot('marker_visibility_report')!=sources
            or any(file_hash(ROOT/'scripts'/name)!=sha for name,sha in source_scripts.items())):
        raise ValueError('diagnostic source, raw output or independent truth changed')
    result.update(provenance=dict(original_native_binary_sha256=build['binary_sha256'],original_manifest_sha256=tasks['manifest_sha256'],
        original_execution_sha256=raw_hash,original_game_job_sha256=file_hash(directory/'game.job.json'),
        original_game_status_sha256=file_hash(directory/'game.job.status.json'),independent_truth_sha256=truth_hash,
        worker_source_binding=proof,diagnostic_source_sha256=sources,diagnostic_scripts_sha256=source_scripts),
        original_navigation_overall=native['overall'])
    if tasks['scope']=='synthetic_controlled_visual_marker_search':result['proxy_control_acceptance']=control_acceptance(result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run','project','out'):parser.add_argument('--'+name,type=Path,required=True)
    for name in ('scene','controls','capture','capture-job'):parser.add_argument('--'+name,type=Path)
    parser.add_argument('--worker-source-commit',required=True,help='full immutable source SHA matching original locked worker bytes')
    args=parser.parse_args();result=evaluate(args)
    with args.out.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result['summary'],indent=2))


if __name__=='__main__':main()
