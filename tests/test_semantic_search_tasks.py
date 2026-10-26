import importlib.util
import json
from pathlib import Path
import sys
import unittest

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))


class SearchTaskTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('semantic_search_tasks',TOOLS/'semantic_search_tasks.py')
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.source=dict(condition='engine_oracle',map='/Game/SpatialLab/Fixture',
            entities=[dict(entity_id='hidden-asset',category='chair',location_cm=[9876,5432,0])],
            episodes=[dict(episode_id='task-a',category='chair',start_cm=[0,0,0],max_sim_s=60,
                           max_path_cm=20000,arrival_tolerance_cm=30,success_radius_cm=300)])
        self.anchors={'task-a':[[1000,0,0]]}

    def test_policy_bundle_omits_hidden_targets_and_oracle_decisions(self):
        bundle=self.module.build_bundle(self.source,self.anchors)
        self.assertEqual(len(bundle['missions']),2)
        self.assertNotIn('hidden-asset',json.dumps(bundle))
        self.assertNotIn('location_cm',json.dumps(bundle))
        self.assertEqual(bundle['missions'][0]['exploration_points_cm'],[[1000,0,0]])
        self.assertEqual(self.module.validate_bundle(bundle),bundle)

    def test_changed_mission_is_rejected_by_digest(self):
        bundle=self.module.build_bundle(self.source,self.anchors)
        bundle['missions'][0]['category']='lamp'
        with self.assertRaises(ValueError):self.module.validate_bundle(bundle)

    def test_anchor_query_export_has_only_start_geometry(self):
        result=self.module.export_start_queries(self.source)
        self.assertEqual(result['queries'],[dict(episode_id='task-a',start_cm=[0,0,0])])
        self.assertNotIn('chair',json.dumps(result))
        self.assertNotIn('hidden-asset',json.dumps(result))

    def test_missing_anchor_episode_is_not_silently_dropped(self):
        with self.assertRaises(ValueError):self.module.build_bundle(self.source,{})

    def test_privileged_extra_fields_fail_even_with_recomputed_digest(self):
        bundle=self.module.build_bundle(self.source,self.anchors)
        bundle['missions'][0]['entities']=self.source['entities']
        bundle['manifest_sha256']=self.module.digest_bundle(bundle)
        with self.assertRaises(ValueError):self.module.validate_bundle(bundle)
