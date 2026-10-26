"""Audit the original physical suite, then explain endpoint-height disagreements."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from support_physics_experiment import audit
from support_physics_goal_report import evaluate
from navigation_build import verify_build,file_hash
from navigation_experiment import read,write
from marker_extent_replay import source_snapshot


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('project','controls','run','out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();directory=args.run.absolute();build=verify_build(ROOT,args.project.absolute())
    sources=source_snapshot('support_physics_goal_report')
    hashes={name:file_hash(directory/name) for name in ('tasks.json','execution.json','game.job.json','game.job.status.json')}
    audited=audit(directory,args.project.absolute(),build,args.controls.absolute())
    result=evaluate(read(directory/'tasks.json'),read(directory/'execution.json'))
    if result['original_native_overall']!=audited['overall'] or result['original_native_by_mode']!=audited['by_mode']:raise ValueError('diagnostic changed original actual scores')
    if any(file_hash(directory/name)!=sha for name,sha in hashes.items()) or source_snapshot('support_physics_goal_report')!=sources:
        raise ValueError('original evidence or evaluator changed during posthoc analysis')
    result.update(original_files_sha256=hashes,evaluation_source_sha256=sources,current_native_binary_sha256=build['binary_sha256'],
        script_sha256=file_hash(Path(__file__)))
    write(args.out,result);print(json.dumps(result['by_mode'],indent=2))


if __name__=='__main__':main()
