"""Audit both complete city horizons before analyzing repeated initial inputs."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'ue5/tools'))
from city_marker_experiment import audit
from compare_marker_horizon_runs import compare
from marker_horizon_tasks import without_horizon
from marker_repeatability import initial_groups
from marker_extent_replay import source_snapshot
from navigation_build import verify_build,file_hash
from navigation_experiment import read,write


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs',nargs=2,type=Path,required=True);parser.add_argument('--controls',nargs=2,type=Path,required=True)
    for name in ('project','scene','capture','capture-job','out'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();build=verify_build(ROOT,args.project.absolute());audited=[];results=[];source=source_snapshot('marker_repeatability')
    for directory,controls in zip(args.runs,args.controls):
        tasks=read(directory/'tasks.json')
        if tasks.get('schema_version')!=3 or tasks.get('planner_variant')!='legacy_centers':raise ValueError('isolated fixed legacy horizon pair required')
        sha=file_hash(directory/'execution.json')
        report=audit(SimpleNamespace(**{**vars(args),'controls':controls}),directory,build)
        audited.append(dict(policy=tasks['goal_horizon_policy'],canonical_manifest_sha256=without_horizon(tasks)['manifest_sha256'],
            native_binary_sha256=build['binary_sha256'],scene_sha256=report['scene_sha256'],report=report))
        groups=initial_groups(read(directory/'execution.json'))
        if file_hash(directory/'execution.json')!=sha or report['raw_execution_sha256']!=sha:raise ValueError('original record changed during repeated-input analysis')
        results.append(dict(policy=tasks['goal_horizon_policy'],native_overall=report['overall'],source_execution_sha256=sha,
            game_job_sha256=file_hash(directory/'game.job.json'),game_status_sha256=file_hash(directory/'game.job.status.json'),initial_inputs=groups))
    compare(audited)
    if source_snapshot('marker_repeatability')!=source:raise ValueError('grouping source changed during analysis')
    sets=[{row['input_without_horizon_sha256'] for group in r['initial_inputs']['groups'] for row in group['rows']} for r in results]
    output=dict(schema_version=1,evidence_kind='audited_city_initial_visual_repeatability_analysis',conditions=results,
        matched_category_input_groups=len(sets[0]&sets[1]),unmatched_category_input_groups=[len(sets[i]-sets[1-i]) for i in range(2)],
        grouping_source_sha256=source,script_sha256=file_hash(Path(__file__)),
        interpretation='full original native outcomes retained; repeated pixels quantify sample correlation and same-condition action consistency, not independent trials, motion determinism or statistical significance')
    write(args.out,output);print(json.dumps([dict(policy=r['policy'],**{k:v for k,v in r['initial_inputs'].items() if k!='groups'}) for r in results],indent=2))


if __name__=='__main__':main()
