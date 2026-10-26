import importlib.util
from pathlib import Path
import unittest


class TaskManifestTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).resolve().parents[1]/'ue5/tools/navigation_tasks.py'
        spec=importlib.util.spec_from_file_location('navigation_tasks',path)
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.probe=dict(condition='engine_navmesh_and_collision_oracle',saved_map='/Game/SpatialLab/Synthetic',
                        probes=[dict(status='ready',nav_projected_cm=[x,800,10]) for x in (0,1000,2000,3000)],
                        navigation=dict(populated_tiles=1),targets=[
            dict(entity_id=str(i),category='chair' if i%2 else 'lamp',location_cm=[i*500,0,0],
                 approaches=[dict(status='ready',nav_projected_cm=[i*500+200,0,10])]) for i in range(8)])

    def test_manifest_has_frozen_deterministic_starts_budgets_and_privilege(self):
        first=self.module.build_manifest(self.probe)
        second=self.module.build_manifest(self.probe)
        self.assertEqual(first,second)
        self.assertEqual(len(first['episodes']),6)
        self.assertEqual(first['condition'],'engine_oracle')
        self.assertEqual(first['policies'],['euclidean_nearest','shortest_reachable'])
        self.assertEqual(first['evidence_kind'],'task_definition_not_execution')
        self.assertEqual({t['category'] for t in first['episodes']},{'chair','lamp'})
        self.assertTrue(all(t['max_sim_s']==60 for t in first['episodes']))

    def test_spawn_inside_a_goal_success_radius_is_not_a_search_task(self):
        self.probe['probes']=[dict(status='ready',nav_projected_cm=[100,0,10])]
        with self.assertRaises(ValueError):
            self.module.build_manifest(self.probe)

    def test_plan_rejects_changed_tasks_and_duplicate_or_missing_runs(self):
        manifest=self.module.build_manifest(self.probe)
        with self.assertRaises(ValueError):
            self.module.validate_plan(manifest,dict(condition='engine_oracle',manifest_sha256='changed',runs=[]))
        with self.assertRaises(ValueError):
            self.module.validate_plan(manifest,dict(condition='engine_oracle',manifest_sha256=self.module.digest_manifest(manifest),runs=[]))

    def test_empty_navmesh_is_rejected(self):
        self.probe['navigation']['populated_tiles']=0
        with self.assertRaises(ValueError):
            self.module.build_manifest(self.probe)

    def test_frozen_manifest_hash_changes_when_task_changes(self):
        first=self.module.build_manifest(self.probe)
        old=self.module.digest_manifest(first)
        first['episodes'][0]['success_radius_cm']=301
        self.assertNotEqual(old,self.module.digest_manifest(first))

    def test_actor_enumeration_order_does_not_change_tasks(self):
        first=self.module.build_manifest(self.probe)
        self.probe['targets'].reverse()
        self.assertEqual(first,self.module.build_manifest(self.probe))


if __name__=='__main__':
    unittest.main()
