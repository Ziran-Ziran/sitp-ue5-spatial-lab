import importlib.util
import json
from pathlib import Path
import sys
import unittest

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))
import test_semantic_observations as observation_fixtures


class SemanticReplayTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('semantic_goal_replay',TOOLS/'semantic_goal_replay.py')
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        fixture=observation_fixtures.ObservationStreamTests()
        fixture.setUp()
        self.records=fixture.records
        self.goals=dict(schema_version=1,condition='engine_local_visibility_supervision',map=fixture.header['map'],
                        manifest_sha256='a'*64,goals=[dict(run_id='run-a',category='chair')])

    def test_goal_export_never_passes_catalog_or_start_positions_to_policy(self):
        manifest=dict(condition='engine_oracle',map='/Game/SpatialLab/Fixture',
                      episodes=[dict(episode_id='task',category='chair',start_cm=[12,34,56])],
                      policies=['euclidean_nearest'],entities=[dict(entity_id='hidden-private',location_cm=[987,654,321])])
        result=self.module.export_goals(manifest)
        self.assertEqual(result['goals'],[dict(run_id='task-euclidean_nearest',category='chair')])
        self.assertNotIn('hidden-private',json.dumps(result))
        self.assertNotIn('start_cm',json.dumps(result))
        self.assertNotIn('entities',result)

    def test_replay_compares_only_current_and_prior_frames(self):
        result=self.module.replay(self.goals,self.records,memory_ttl_s=5)
        self.assertTrue(result['finished'])
        self.assertEqual(result['by_policy']['frame_only']['goal_available_frames'],1)
        self.assertEqual(result['by_policy']['temporal_memory']['goal_available_frames'],2)
        self.assertIsNone(result['selections'][2]['temporal_memory'])
        self.assertEqual(result['evidence_kind'],'offline_semantic_selection_replay')

    def test_missing_observation_run_stays_in_run_denominator(self):
        self.goals['goals'].append(dict(run_id='missing',category='lamp'))
        result=self.module.replay(self.goals,self.records,memory_ttl_s=5)
        self.assertFalse(result['finished'])
        self.assertEqual(result['by_policy']['temporal_memory']['runs'],2)
        self.assertEqual(result['by_policy']['temporal_memory']['runs_with_available_goal'],1)
        self.assertEqual(result['runs'][1]['observation_failure_reason'],'missing_observation_episode')

    def test_full_truth_task_or_wrong_stream_hash_cannot_be_policy_input(self):
        self.goals['entities']=[dict(entity_id='hidden')]
        with self.assertRaises(ValueError): self.module.replay(self.goals,self.records)
        del self.goals['entities']
        self.goals['manifest_sha256']='b'*64
        with self.assertRaises(ValueError): self.module.replay(self.goals,self.records)


if __name__=='__main__':
    unittest.main()
