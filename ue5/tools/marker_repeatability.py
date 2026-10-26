"""Initial visual-input grouping after native audit; no new independent trials."""
from collections import Counter,defaultdict
import copy
import hashlib
import json
import re


def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def initial_groups(record):
    groups=defaultdict(list);pixels_only=set();missing=[];seen=set()
    if not 1<=len(record['runs'])<=200:raise ValueError('bounded complete source run set required')
    for run in record['runs']:
        if run['run_id'] in seen:raise ValueError('duplicate source run')
        seen.add(run['run_id']);events=[e for e in run['execution']['actions'] if e.get('step_id')==0]
        if len(events)>1:raise ValueError('duplicate initial observation')
        if not events or events[0]['status']!='complete':missing.append(run['run_id']);continue
        event=events[0];request=event['request'];response=copy.deepcopy(event['response'])
        if any(request.get(k)!=response.get(k) for k in ('attempt_id','run_id','step_id','t_sim_s','category','consecutive_empty_views','schema_version','planner_variant','goal_horizon_policy')) or request['run_id']!=run['run_id']:
            raise ValueError('original request/response identity and condition must agree')
        pixels={k:v for k,v in response['input_files_sha256'].items() if k.endswith('.png')}
        if set(pixels)!={v+'/'+k+'.png' for v in ('level','down45','nadir') for k in ('rgb','depth')} or any(not re.fullmatch('[0-9a-f]{64}',sha) for sha in pixels.values()):
            raise ValueError('all six original pixel hashes required')
        pixels_only.add(digest(pixels))
        condition={key:request.get(key) for key in ('schema_version','category','consecutive_empty_views','planner_variant','goal_horizon_policy')}
        key=digest(dict(pixels=pixels,condition=condition))
        for field in ('attempt_id','run_id','step_id','t_sim_s','perception_and_planning_wall_s'):response.pop(field,None)
        if response.get('geometry') is not None:response['geometry']['planning'].pop('planner_wall_s',None)
        response['input_files_sha256']=pixels
        cross_condition=dict(condition);cross_condition.pop('goal_horizon_policy')
        groups[key].append(dict(run_id=run['run_id'],response_sha256=digest(response),
            input_without_horizon_sha256=digest(dict(pixels=pixels,condition=cross_condition))))
    packed=[dict(group_sha256=key,rows=rows) for key,rows in sorted(groups.items())]
    return dict(full_runs=len(seen),missing_initial_observations=missing,distinct_initial_six_png_groups=len(pixels_only),
        distinct_category_input_groups=len(groups),category_group_sizes=dict(Counter(len(rows) for rows in groups.values())),groups=packed,
        conflicting_same_input_responses=[group for group in packed if len({r['response_sha256'] for r in group['rows']})!=1],
        interpretation='identical initial visual inputs under a fixed query condition; correlation, not independent episodes, physical repeatability or generalization; native scores unchanged')
