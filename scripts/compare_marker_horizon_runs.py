"""Rebuild both full city goal horizons with matching current marker controls."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from city_marker_experiment import audit
from navigation_build import verify_build,file_hash
from navigation_experiment import read
from marker_goal_horizon import POLICIES
from marker_horizon_tasks import without_horizon


def compare(audited):
    if len(audited)!=2 or {row['policy'] for row in audited}!=set(POLICIES):raise ValueError('both unique predeclared goal horizons required')
    for key in ('canonical_manifest_sha256','native_binary_sha256','scene_sha256'):
        if audited[0][key]!=audited[1][key]:raise ValueError('horizon pair changes fixed tasks/native/scene')
    return dict(schema_version=1,evidence_kind='actual_city_observed_marker_goal_horizon_comparison',conditions=audited,
        interpretation='same complete controlled-color tasks/legacy planner/current native/scene; lookahead alone changes; all failures retained, not furniture recognition or generalization')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',nargs=2,type=Path,required=True);parser.add_argument('--controls',nargs=2,type=Path,required=True)
    for name in ('project','scene','capture','capture-job','out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();build=verify_build(ROOT,args.project.absolute());audited=[]
    for directory,controls in zip(args.runs,args.controls):
        tasks=read(directory/'tasks.json')
        if tasks.get('schema_version')!=3 or tasks.get('planner_variant')!='legacy_centers':raise ValueError('version-three isolated horizon comparison requires fixed legacy planner')
        report=audit(SimpleNamespace(**{**vars(args),'controls':controls}),directory,build)
        audited.append(dict(policy=tasks['goal_horizon_policy'],canonical_manifest_sha256=without_horizon(tasks)['manifest_sha256'],
            native_binary_sha256=build['binary_sha256'],scene_sha256=report['scene_sha256'],report=report,
            game_job_sha256=file_hash(directory/'game.job.json'),game_status_sha256=file_hash(directory/'game.job.status.json')))
    result=compare(audited)
    with args.out.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps([dict(policy=row['policy'],overall=row['report']['overall']) for row in audited],indent=2))


if __name__=='__main__':main()
