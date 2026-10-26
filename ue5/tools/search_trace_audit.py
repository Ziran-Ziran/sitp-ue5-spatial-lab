"""Posthoc sequential audit of saved actions against the same observed-only policy.

This replays decisions, not physics or counterfactual trajectories. The policy is
given one saved frame and its corresponding action feedback at a time.
"""
import math
from semantic_search_policy import SearchPolicy


def equivalent(a,b):
    if isinstance(a,dict) and isinstance(b,dict):
        return set(a)==set(b) and all(equivalent(a[key],b[key]) for key in a)
    if isinstance(a,list) and isinstance(b,list):
        return len(a)==len(b) and all(equivalent(x,y) for x,y in zip(a,b))
    if isinstance(a,(int,float)) and not isinstance(a,bool) and isinstance(b,(int,float)) and not isinstance(b,bool):
        return math.isfinite(a) and math.isfinite(b) and abs(a-b)<=1e-5
    return type(a) is type(b) and a==b


def audit(mission,state,frames):
    trace=state.get('policy_trace',[])
    if not isinstance(trace,list):return dict(passed=False,reason='malformed_trace')
    steps={}
    last=-1
    for row in trace:
        index=row.get('frame_id')
        if isinstance(index,bool) or not isinstance(index,int) or index<=last or index not in frames:
            return dict(passed=False,reason='unexpected_or_reordered_policy_frame')
        last=index
        steps[index]=row
    policy=SearchPolicy(mission)
    allowed_no_decision={'timeout','path_budget','wall_timeout','agent_lost','observation_io_failure','policy_step_failed'}
    for index,frame in sorted(frames.items()):
        if index not in steps:
            if index==max(frames) and state['status'] in allowed_no_decision:continue
            return dict(passed=False,reason='missing_policy_frame')
        row=steps[index]
        try:
            if abs(row['t_sim_s']-frame['t_sim_s'])>1e-4:
                return dict(passed=False,reason='policy_clock_mismatch')
            expected=policy.step(frame,row['feedback'])
        except (ValueError,KeyError,TypeError):
            return dict(passed=False,reason='invalid_policy_feedback_or_state')
        if not equivalent(expected,row['action']):
            return dict(passed=False,reason='action_not_reproduced_from_observations')
    return dict(passed=True,reason=None,policy_steps=len(trace))
