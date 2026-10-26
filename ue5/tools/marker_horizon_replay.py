"""Same saved RGB-D requests, two explicit lookaheads; no simulated arrivals."""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path
from marker_goal_horizon import POLICIES
from marker_extent_replay import source_snapshot
from rendered_input_report import file_hash
from visual_marker_depth_policy import decide


def canonical(response):
    result=copy.deepcopy(response);result.pop('perception_and_planning_wall_s',None)
    if result['geometry'] is not None:result['geometry']['planning'].pop('planner_wall_s',None)
    return result


def evaluate_record(path):
    path=Path(path);raw_hash=file_hash(path);sources=source_snapshot('marker_horizon_replay');raw=json.loads(path.read_text(encoding='utf-8-sig'))
    if not 1<=len(raw['runs'])<=200:raise ValueError('bounded complete saved run set required')
    rows=[];seen=set()
    for run in raw['runs']:
        if run['run_id'] in seen or len(run['execution']['actions'])>12:raise ValueError('duplicate run or unbounded saved observations')
        seen.add(run['run_id'])
        for event in run['execution']['actions']:
            row=dict(run_id=run['run_id'],step_id=event['step_id'],status=event['status'],models={});rows.append(row)
            if event['status']!='complete':continue
            request=event['request']
            if request['schema_version']!=2 or request['run_id']!=run['run_id'] or request['category']!=run['input']['category']:
                raise ValueError('original version-two fixed planner request required')
            baseline=canonical(decide(request))
            if baseline!=canonical(event['response']):raise ValueError('old action differs from original saved pixels')
            for policy in POLICIES:
                answer=decide(dict(request,schema_version=3,goal_horizon_policy=policy));row['models'][policy]=canonical(answer)
                if policy==POLICIES[0]:
                    normalized=canonical(answer);normalized.pop('goal_horizon_policy');normalized['schema_version']=2
                    if normalized!=baseline:raise ValueError('three-meter version-three baseline changes legacy decision')
    if file_hash(path)!=raw_hash or source_snapshot('marker_horizon_replay')!=sources:raise ValueError('frozen record or worker changed during replay')
    complete=[row for row in rows if row['models']];summary=dict(source_runs=len(seen),observations=len(rows),complete_observations=len(complete))
    for policy in POLICIES:
        selected=[row['models'][policy] for row in complete]
        summary[policy]=dict(actions=dict(Counter(item['action'] for item in selected)),reasons=dict(Counter(item['reason'] for item in selected)))
    summary['action_transitions']=dict(Counter(row['models'][POLICIES[0]]['action']+'->'+row['models'][POLICIES[1]]['action'] for row in complete))
    return dict(schema_version=1,evidence_kind='offline_same_saved_request_goal_horizon_comparison',source_execution_sha256=raw_hash,
        worker_source_sha256=sources,summary=summary,observations=rows,
        interpretation='same historical pixels, headings and scan counters, not a counterfactual motion trace; route availability is not arrival, semantic success or new native execution')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execution',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();result=evaluate_record(args.execution)
    with args.out.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result['summary'],indent=2))


if __name__=='__main__':main()
