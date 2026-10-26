import importlib.util
from pathlib import Path
import sys
import unittest


TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))


class NavigationReportTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('navigation_report',TOOLS/'navigation_report.py')
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        from navigation_tasks import digest_manifest
        self.task=dict(episode_id='fixed',category='chair',success_radius_cm=300,start_cm=[1000,0,0])
        self.manifest=dict(condition='engine_oracle',map='/Game/SpatialLab/Intended',policies=['euclidean_nearest','shortest_reachable'],
                           episodes=[self.task],entities=[dict(entity_id='chair-1',category='chair',location_cm=[0,0,0])])
        self.record=dict(condition='engine_oracle',map='/Game/SpatialLab/Intended',manifest_sha256=digest_manifest(self.manifest),
                         evidence_kind='ue_game_world_character_movement',finished=False,runs=[])

    def test_missing_execution_keeps_every_frozen_run_in_denominator(self):
        report=self.module.evaluate(self.manifest,self.record)
        self.assertEqual(report['overall']['episodes'],2)
        self.assertEqual(report['overall']['success_rate'],0)
        self.assertEqual(report['overall']['failures'],{'missing_execution':2})
        self.assertEqual(report['overall']['collision_observed_episodes'],0)

    def test_success_is_scored_independently_and_missing_peer_is_failure(self):
        self.record['runs']=[dict(run_id='fixed-euclidean_nearest',policy='euclidean_nearest',task=self.task,
            decision=dict(selection_wall_s=.1),execution=dict(status='arrived',entity_id='chair-1',
            final_position_cm=[100,0,0],trajectory_cm=[[1000,0,0],[100,0,0]],wall_s=2,collisions=1))]
        report=self.module.evaluate(self.manifest,self.record)
        self.assertEqual(report['overall']['success_rate'],.5)
        self.assertEqual(report['by_policy']['euclidean_nearest']['total_path_length_cm'],900)
        self.assertEqual(report['by_policy']['shortest_reachable']['failures'],{'missing_execution':1})

    def test_duplicate_results_are_not_silently_chosen(self):
        row=dict(run_id='fixed-euclidean_nearest',policy='euclidean_nearest',task=self.task,execution={})
        self.record['runs']=[row,row]
        with self.assertRaises(ValueError):
            self.module.evaluate(self.manifest,self.record)

    def test_changed_frozen_task_or_hash_is_rejected(self):
        self.record['manifest_sha256']='incorrect'
        with self.assertRaises(ValueError):
            self.module.evaluate(self.manifest,self.record)

    def test_another_map_cannot_succeed_under_the_same_manifest_hash(self):
        self.record['map']='/Game/SpatialLab/WrongMap'
        with self.assertRaises(ValueError):
            self.module.evaluate(self.manifest,self.record)


if __name__=='__main__':
    unittest.main()
