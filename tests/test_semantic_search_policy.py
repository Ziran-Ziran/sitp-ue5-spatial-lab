import copy
import importlib.util
from pathlib import Path
import sys
import unittest

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))


class SemanticSearchPolicyTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('semantic_search_policy',TOOLS/'semantic_search_policy.py')
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.mission=dict(run_id='run-a',category='chair',policy='frame_only',start_cm=[0,0,0],
            max_sim_s=60,max_path_cm=20000,arrival_tolerance_cm=30,success_radius_cm=300,
            exploration_points_cm=[[1000,0,0],[0,1000,0]])

    def frame(self,index=0,t=0,entities=(),feet=None):
        return dict(record_type='frame',run_id='run-a',frame_id=index,t_sim_s=t,t_monotonic_s=t+.1,
            observer_position_cm=feet or [0,0,0],sensor_eye_cm=[0,0,150],entities=list(entities))

    def entity(self,name='chair-a',xyz=None):
        return dict(entity_id=name,category='chair',position_cm=xyz or [1500,0,0],visible_sample_count=1)

    def feedback(self,action,state='moving'):
        return dict(action_id=action['action_id'],state=state)

    def test_no_observation_explores_without_inventing_target(self):
        policy=self.module.SearchPolicy(self.mission)
        action=policy.step(self.frame(),dict(action_id=None,state='none'))
        self.assertEqual(action['kind'],'move')
        self.assertEqual(action['reason'],'explore')
        self.assertIsNone(action['entity_id'])
        self.assertEqual(action['goal_cm'],[1000,0,0])

    def test_observed_target_interrupts_exploration(self):
        policy=self.module.SearchPolicy(self.mission)
        explore=policy.step(self.frame(),dict(action_id=None,state='none'))
        target=policy.step(self.frame(1,1,[self.entity()]),self.feedback(explore))
        self.assertEqual(target['reason'],'observed_goal')
        self.assertEqual(target['entity_id'],'chair-a')
        self.assertEqual(target['source_frame_id'],1)
        self.assertEqual(target['goal_cm'],[1320,0,0])
        self.assertEqual(target['entity_position_cm'],[1500,0,0])

    def test_frame_only_cancels_lost_goal_but_memory_keeps_it(self):
        for name,expected in [('frame_only','cancel'),('temporal_memory','hold')]:
            mission=dict(self.mission,policy=name)
            policy=self.module.SearchPolicy(mission)
            move=policy.step(self.frame(entities=[self.entity()]),dict(action_id=None,state='none'))
            action=policy.step(self.frame(1,1),self.feedback(move))
            self.assertEqual(action['kind'],expected)

    def test_unreachable_observed_goal_is_not_retried_forever(self):
        policy=self.module.SearchPolicy(self.mission)
        move=policy.step(self.frame(entities=[self.entity()]),dict(action_id=None,state='none'))
        action=policy.step(self.frame(1,1,[self.entity()]),self.feedback(move,'rejected'))
        self.assertEqual(action['reason'],'explore')
        self.assertIn('chair-a',policy.rejected_entities)

    def test_reaching_observed_category_emits_candidate_for_independent_score(self):
        policy=self.module.SearchPolicy(self.mission)
        action=policy.step(self.frame(entities=[self.entity(xyz=[200,0,0])]),dict(action_id=None,state='none'))
        self.assertEqual(action['kind'],'stop')
        self.assertEqual(action['reason'],'candidate_reached')
        self.assertEqual(action['entity_id'],'chair-a')

    def test_exhaustion_retains_failed_exploration_and_stops(self):
        policy=self.module.SearchPolicy(self.mission)
        a=policy.step(self.frame(),dict(action_id=None,state='none'))
        b=policy.step(self.frame(1,1),self.feedback(a,'rejected'))
        result=policy.step(self.frame(2,2),self.feedback(b,'blocked'))
        self.assertEqual(result['reason'],'exploration_exhausted')

    def test_mission_catalog_and_truth_feedback_are_rejected(self):
        for key in ('entities','target_position_cm','truth'):
            with self.assertRaises(ValueError): self.module.SearchPolicy(dict(self.mission,**{key:[]}))
        policy=self.module.SearchPolicy(self.mission)
        with self.assertRaises(ValueError):
            policy.step(self.frame(),dict(action_id=None,state='none',target_id='hidden'))

    def test_stale_feedback_fails_before_memory_is_mutated(self):
        policy=self.module.SearchPolicy(self.mission)
        a=policy.step(self.frame(),dict(action_id=None,state='none'))
        with self.assertRaises(ValueError):
            policy.step(self.frame(1,1,[self.entity()]),dict(action_id=a['action_id']+1,state='moving'))
        self.assertEqual(policy.memory.frame_id,0)

    def test_mutating_future_mission_or_frame_does_not_rewrite_actions(self):
        policy=self.module.SearchPolicy(self.mission)
        a=policy.step(self.frame(),dict(action_id=None,state='none'))
        self.mission['exploration_points_cm'][0][0]=9999
        self.assertEqual(a['goal_cm'],[1000,0,0])
