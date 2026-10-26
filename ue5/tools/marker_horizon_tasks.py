"""Bind an explicit lookahead ablation over an unchanged schema-two task set."""
import copy
from marker_goal_horizon import POLICIES,EXECUTION_PREFIX_M
from online_depth_tasks import digest,SETTINGS


def with_horizon(tasks,policy):
    if tasks.get('schema_version')!=2 or policy not in POLICIES or SETTINGS['receding_route_m']!=EXECUTION_PREFIX_M:
        raise ValueError('explicit horizon over a fixed planner task set required')
    result=copy.deepcopy(tasks);result.update(schema_version=3,goal_horizon_policy=policy)
    result['runs']=[dict(row,goal_horizon_policy=policy) for row in result['runs']]
    result['manifest_sha256']=digest(result);return result


def without_horizon(tasks):
    policy=tasks.get('goal_horizon_policy')
    if (tasks.get('schema_version')!=3 or policy not in POLICIES or tasks.get('manifest_sha256')!=digest(tasks)
            or not isinstance(tasks.get('runs'),list) or not 1<=len(tasks['runs'])<=200
            or any(row.get('goal_horizon_policy')!=policy for row in tasks['runs'])):
        raise ValueError('version-three top/run horizon must be fixed and identical')
    result=copy.deepcopy(tasks);result.pop('goal_horizon_policy');result['schema_version']=2
    for row in result['runs']:row.pop('goal_horizon_policy')
    result['manifest_sha256']=digest(result);return result
