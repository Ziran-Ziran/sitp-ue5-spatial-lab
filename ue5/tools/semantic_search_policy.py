"""Observed-only search decisions; no UE imports, target catalog or truth input.

Exploration anchors are explicit engine-geometry supervision, frozen independently
of target locations. The same implementation runs in CPU tests and UE's embedded
Python. Selection success is only a candidate for independent posthoc evaluation.
"""
import copy
import json
import math
from pathlib import Path
from semantic_memory import CATEGORIES,CONDITION,SemanticMemory,finite,position


MISSION_KEYS={'run_id','category','policy','start_cm','max_sim_s','max_path_cm',
              'arrival_tolerance_cm','success_radius_cm','exploration_points_cm'}
POLICIES={'frame_only':0,'temporal_memory':5}


def validate_mission(mission):
    if set(mission)!=MISSION_KEYS or mission['category'] not in CATEGORIES or mission['policy'] not in POLICIES:
        raise ValueError('mission accepts goal category and geometry anchors only')
    if not isinstance(mission['run_id'],str) or not mission['run_id']:
        raise ValueError('invalid run ID')
    position(mission['start_cm'])
    for field,lower,upper in [('max_sim_s',0,120),('max_path_cm',0,100000),
                               ('arrival_tolerance_cm',0,100),('success_radius_cm',0,300)]:
        if not lower<finite(mission[field])<=upper: raise ValueError('invalid bounded mission budget')
    anchors=mission['exploration_points_cm']
    if not isinstance(anchors,list) or len(anchors)>64:
        raise ValueError('bounded geometry anchors required')
    normalized=[position(point) for point in anchors]
    if len(set(map(tuple,normalized)))!=len(normalized): raise ValueError('duplicate exploration anchors')
    if any(math.dist(point,mission['start_cm'])>5000 for point in normalized):
        raise ValueError('exploration anchor beyond fixed local radius')
    return copy.deepcopy(mission)


class SearchPolicy:
    def __init__(self,mission):
        self.mission=validate_mission(mission)
        self.memory=SemanticMemory(mission['run_id'],POLICIES[mission['policy']],condition=CONDITION)
        self.active=None
        self.last_action_id=None
        self.attempted_anchors=set()
        self.rejected_entities=set()
        self.commands=0
        self.finished=False

    def issue(self,kind,reason,*,goal=None,entity=None,anchor=None):
        action=dict(action_id=self.commands,kind=kind,reason=reason,
                    goal_cm=list(goal) if goal is not None else None,
                    entity_id=entity['entity_id'] if entity else None,
                    entity_position_cm=list(entity['position_cm']) if entity else None,
                    source_frame_id=entity['source_frame_id'] if entity else None,
                    anchor_index=anchor)
        self.commands+=1
        self.last_action_id=action['action_id']
        self.active=action if kind=='move' else None
        self.finished=kind=='stop'
        return copy.deepcopy(action)

    def step(self,frame,feedback):
        if self.finished: raise ValueError('policy already stopped')
        if (set(feedback)!={'action_id','state'} or feedback['action_id']!=self.last_action_id
                or feedback['state'] not in {'none','moving','arrived','rejected','blocked','cancelled'}
                or (self.last_action_id is None and feedback['state']!='none')):
            raise ValueError('stale or privileged feedback')
        self.memory.update(frame)
        previous=self.active
        terminal=feedback['state'] in {'arrived','rejected','blocked','cancelled'}
        if previous and terminal:
            self.active=None
            if previous['entity_id'] and (feedback['state']!='arrived'
                    or math.dist(self.memory.observer,previous['entity_position_cm'])>self.mission['success_radius_cm']):
                self.rejected_entities.add(previous['entity_id'])
        candidates=[entity for entity in self.memory.entities.values()
                    if entity['category']==self.mission['category'] and entity['entity_id'] not in self.rejected_entities]
        target=min(candidates,key=lambda entity:(math.dist(self.memory.observer,entity['position_cm']),entity['entity_id'])) if candidates else None
        if target and math.dist(self.memory.observer,target['position_cm'])<=self.mission['success_radius_cm']:
            return self.issue('stop','candidate_reached',goal=target['position_cm'],entity=target)
        if self.commands>=64:
            return self.issue('stop','action_budget')
        if target:
            if self.active and self.active['entity_id']==target['entity_id']:
                return dict(action_id=self.last_action_id,kind='hold')
            distance=math.dist(self.memory.observer,target['position_cm'])
            offset=self.mission['success_radius_cm']*.6
            approach=[p+(observer-p)*offset/distance for p,observer in zip(target['position_cm'],self.memory.observer)]
            return self.issue('move','observed_goal',goal=approach,entity=target)
        if self.active and self.active['entity_id']:
            return self.issue('cancel','goal_observation_expired')
        if self.active:
            return dict(action_id=self.last_action_id,kind='hold')
        remaining=[(index,point) for index,point in enumerate(self.mission['exploration_points_cm'])
                   if index not in self.attempted_anchors]
        if not remaining:
            return self.issue('stop','exploration_exhausted')
        index,point=min(remaining,key=lambda pair:(math.dist(self.memory.observer,pair[1]),pair[0]))
        self.attempted_anchors.add(index)
        return self.issue('move','explore',goal=point,anchor=index)


def dispatch(policy,frame_json,feedback_json,response_path,attempt_id):
    """Synchronous native bridge. Never loads UE, annotations or future records."""
    frame=json.loads(frame_json)
    action=policy.step(frame,json.loads(feedback_json))
    result=dict(attempt_id=attempt_id,run_id=policy.mission['run_id'],frame_id=frame['frame_id'],action=action)
    path=Path(response_path)
    # One exclusive response per synchronous request; stale files fail closed.
    with path.open('x',encoding='utf-8') as output:
        json.dump(result,output,allow_nan=False)
