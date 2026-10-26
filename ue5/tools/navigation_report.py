"""Score all frozen oracle episodes, including missing/failed executions.

Raw execution reports and engine truth stay local. This tool does not run UE,
modify trajectories or feed scoring truth back into planning or execution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from navigation_tasks import digest_manifest
from semantic_navigation import score_episode,summarize_scores


def evaluate(manifest,record):
    if (record.get('condition')!='engine_oracle' or manifest.get('condition')!='engine_oracle'
            or record.get('manifest_sha256')!=digest_manifest(manifest)
            or not manifest.get('map','').startswith('/Game/SpatialLab/') or record.get('map')!=manifest['map']):
        raise ValueError('execution condition or frozen manifest hash mismatch')
    if record.get('evidence_kind')!='ue_game_world_character_movement':
        raise ValueError('this report requires actual Game-world movement evidence')
    tasks={t['episode_id']:t for t in manifest['episodes']}
    expected={episode+'-'+policy:(task,policy) for episode,task in tasks.items() for policy in manifest['policies']}
    rows={}
    for row in record['runs']:
        key=row['run_id']
        if key in rows or key not in expected:
            raise ValueError('duplicate or unexpected execution run')
        task,policy=expected[key]
        if row.get('task')!=task or row.get('policy')!=policy:
            raise ValueError('execution changes frozen task or policy')
        rows[key]=row
    scores=[]
    for key,(task,policy) in expected.items():
        row=rows.get(key,{})
        execution=row.get('execution',dict(status='missing_execution',trajectory_cm=[],collisions=None,wall_s=None))
        score=score_episode(task,execution,manifest['entities'])
        score.update(run_id=key,policy=policy,selection_wall_s=row.get('decision',{}).get('selection_wall_s'),
                     path_queries=row.get('decision',{}).get('path_queries'),sim_s=execution.get('sim_s'))
        scores.append(score)
    def summary(group):
        result=summarize_scores(group)
        planning=[s['selection_wall_s'] for s in group if s['selection_wall_s'] is not None]
        combined=[s['selection_wall_s']+s['wall_s'] for s in group
                  if s['selection_wall_s'] is not None and s['wall_s'] is not None]
        queries=[s['path_queries'] for s in group if s['path_queries'] is not None]
        result.update(planning_latency_observed_episodes=len(planning),
                      mean_planning_wall_s=sum(planning)/len(planning) if planning else None,
                      end_to_end_latency_observed_episodes=len(combined),
                      mean_planning_plus_execution_wall_s=sum(combined)/len(combined) if combined else None,
                      total_path_queries=sum(queries),path_query_observed_episodes=len(queries))
        return result
    return dict(schema_version=1,condition='engine_oracle',evidence_kind=record['evidence_kind'],
                manifest_sha256=digest_manifest(manifest),
                finished=record.get('finished') is True and len(rows)==len(expected),
                interpretation='fixed local engine-label task; no visual recognition or cross-scene generalization claim',
                latency_definition='wall time for selection calls plus Game-world episodes; excludes engine startup; simulated time is separate',
                overall=summary(scores),
                by_policy={p:summary([s for s in scores if s['policy']==p]) for p in manifest['policies']},
                scores=scores)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('tasks',type=Path)
    parser.add_argument('execution',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=evaluate(json.loads(args.tasks.read_text(encoding='utf-8-sig')),
                    json.loads(args.execution.read_text(encoding='utf-8-sig')))
    result['raw_execution_sha256']=hashlib.sha256(args.execution.read_bytes()).hexdigest()
    with args.out.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result['by_policy'],indent=2))
