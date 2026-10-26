"""Synthetic, metric-centimeter oracle fixtures; no imported scene data."""
import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'ue5/tools/semantic_navigation.py'


class SemanticNavigationTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('semantic_navigation', SCRIPT)
        self.nav = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.nav)
        self.entities = [dict(entity_id='near',category='chair',location_cm=[100,0,0],
                              approaches_cm=[[100,0,0]],source='engine_actor_name_oracle'),
                         dict(entity_id='far',category='chair',location_cm=[300,0,0],
                              approaches_cm=[[300,0,0]],source='engine_actor_name_oracle')]

    def query(self, start, goal):
        if goal[0] == 100:
            return dict(status='unreachable',points_cm=[])
        return dict(status='complete',points_cm=[start,goal])

    def test_euclidean_nearest_keeps_unreachable_failure(self):
        result = self.nav.choose_target(self.entities,[0,0,0],'chair','euclidean_nearest',self.query)
        self.assertEqual(result['entity_id'],'near')
        self.assertEqual(result['status'],'unreachable')
        self.assertEqual(result['path_queries'],1)

    def test_shortest_reachable_can_choose_farther_entity(self):
        result = self.nav.choose_target(self.entities,[0,0,0],'chair','shortest_reachable',self.query)
        self.assertEqual(result['entity_id'],'far')
        self.assertEqual(result['path_length_cm'],300)
        self.assertEqual(result['path_queries'],2)
        self.assertEqual(result['candidate_failures'],{'unreachable':1})

    def test_partial_path_is_never_accepted_as_reachable(self):
        result = self.nav.choose_target(self.entities,[0,0,0],'chair','shortest_reachable',
                                      lambda s,g:dict(status='partial',points_cm=[s,[50,0,0]]))
        self.assertEqual(result['status'],'no_reachable_target')
        self.assertEqual(result['candidate_failures'],{'partial':2})

    def test_tie_is_independent_of_actor_enumeration(self):
        a = dict(self.entities[0],entity_id='a',approaches_cm=[[200,0,0]])
        b = dict(a,entity_id='b')
        query = lambda s,g:dict(status='complete',points_cm=[s,g])
        self.assertEqual(self.nav.choose_target([b,a],[0,0,0],'chair','shortest_reachable',query)['entity_id'],'a')

    def test_nonfinite_or_wrong_endpoint_path_is_invalid(self):
        for points in ([[0,0,0],[float('nan'),0,0]], [[0,0,0],[20,0,0]]):
            result = self.nav.choose_target(self.entities,[0,0,0],'chair','shortest_reachable',
                                          lambda s,g:dict(status='complete',points_cm=points))
            self.assertEqual(result['status'],'no_reachable_target')
            self.assertEqual(result['candidate_failures'],{'invalid_path':2})

    def test_missing_category_is_explicit(self):
        result = self.nav.choose_target(self.entities,[0,0,0],'fountain','euclidean_nearest',self.query)
        self.assertEqual(result['status'],'category_absent')
        self.assertEqual(result['path_queries'],0)

    def test_engine_labels_cannot_be_called_visual_estimates(self):
        with self.assertRaises(ValueError):
            self.nav.choose_target(self.entities,[0,0,0],'chair','shortest_reachable',self.query,
                                   condition='visual_semantics')

    def test_score_uses_final_pose_and_truth_not_claimed_success(self):
        task = dict(episode_id='e',category='chair',success_radius_cm=50)
        execution = dict(status='success',entity_id='near',final_position_cm=[0,0,0],
                         trajectory_cm=[[0,0,0],[25,0,0],[0,0,0]],collisions=1,wall_s=2)
        score = self.nav.score_episode(task,execution,self.entities)
        self.assertFalse(score['success'])
        self.assertEqual(score['failure_reason'],'outside_goal_radius')
        self.assertEqual(score['path_length_cm'],50)
        execution['final_position_cm']=[100,0,0]
        execution['trajectory_cm'].append([100,0,0])
        self.assertTrue(self.nav.score_episode(task,execution,self.entities)['success'])

    def test_wrong_category_cannot_succeed(self):
        task = dict(episode_id='e',category='lamp',success_radius_cm=500)
        result = self.nav.score_episode(task,dict(status='arrived',entity_id='near',
                         final_position_cm=[100,0,0],trajectory_cm=[[100,0,0]],collisions=0,wall_s=1),self.entities)
        self.assertFalse(result['success'])
        self.assertEqual(result['failure_reason'],'wrong_category')

    def test_all_attempts_and_unknown_collisions_are_preserved_in_summary(self):
        summary = self.nav.summarize_scores([
            dict(success=True,collisions=0,path_length_cm=100,wall_s=2,failure_reason=None),
            dict(success=False,collisions=None,path_length_cm=30,wall_s=5,failure_reason='blocked')])
        self.assertEqual(summary['episodes'],2)
        self.assertEqual(summary['success_rate'],.5)
        self.assertEqual(summary['collision_observed_episodes'],1)
        self.assertEqual(summary['failures'],{'blocked':1})


if __name__ == '__main__':
    unittest.main()
