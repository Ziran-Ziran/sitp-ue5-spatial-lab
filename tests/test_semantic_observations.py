import copy
import importlib.util
import json
from pathlib import Path
import sys
import unittest

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))


class ObservationStreamTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('semantic_observation_stream',TOOLS/'semantic_observation_stream.py')
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.header=dict(record_type='stream',schema_version=1,condition='engine_local_visibility_supervision',
            semantic_source='engine_actor_label',pose_source='engine_ground_truth',
            route_source='global_engine_oracle_selected_routes',
            visibility_method='panoramic_range_three_bbox_samples_first_hit_target',units='centimeters',
            coordinate_system='UE left-handed X-forward Y-right Z-up',map='/Game/SpatialLab/Fixture',
            manifest_sha256='a'*64,attempt_id='12345678-1234-4234-8234-123456789abc',
            range_cm=2000,sample_period_sim_s=.1,eye_height_cm=150)
        self.entity=dict(entity_id='observed-chair',category='chair',position_cm=[100,0,0],visible_sample_count=1)
        self.records=[self.header,dict(record_type='episode_start',run_id='run-a')]
        for index,t in enumerate((0,1,6)):
            self.records.append(dict(record_type='frame',run_id='run-a',frame_id=index,t_sim_s=t,t_monotonic_s=t+.1,
                observer_position_cm=[0,0,0],sensor_eye_cm=[0,0,150],entities=[self.entity] if index==0 else []))
        self.records.extend([dict(record_type='episode_end',run_id='run-a',status='arrived',frames=3),
                             dict(record_type='stream_end',finished=True,episodes=1,frames=3)])

    def validate(self):
        return list(self.module.iter_validated(self.records))

    def test_complete_stream_preserves_empty_frames(self):
        self.assertEqual(len(self.validate()),7)
        self.assertEqual(sum(r.get('entities')==[] for r in self.validate()),2)

    def test_observation_dependent_engine_geometry_route_is_explicit(self):
        self.header['route_source']='observation_dependent_engine_navmesh_geometry'
        self.assertEqual(len(self.validate()),7)

    def test_catalog_or_truth_fields_are_not_allowed(self):
        self.records[2]['all_targets']=[dict(entity_id='hidden',position_cm=[900,0,0])]
        with self.assertRaises(ValueError): self.validate()

    def test_incomplete_or_wrong_count_stream_is_rejected(self):
        self.records.pop()
        with self.assertRaises(ValueError): self.validate()
        self.records.append(dict(record_type='stream_end',finished=True,episodes=1,frames=2))
        with self.assertRaises(ValueError): self.validate()

    def test_undisclosed_global_oracle_source_is_rejected(self):
        self.header['condition']='visual_semantics'
        with self.assertRaises(ValueError): self.validate()

    def test_range_and_eye_reference_point_are_checked(self):
        self.entity['position_cm']=[2500,0,0]
        with self.assertRaises(ValueError): self.validate()
        self.entity['position_cm']=[100,0,0]
        self.records[2]['sensor_eye_cm']=[0,0,0]
        with self.assertRaises(ValueError): self.validate()

    def test_records_are_consumed_lazily_without_looking_ahead(self):
        consumed=[]
        def source():
            for index,row in enumerate(self.records):
                consumed.append(index)
                yield row
        stream=self.module.iter_validated(source())
        self.assertEqual(next(stream)['record_type'],'stream')
        self.assertEqual(consumed,[0])
        next(stream)
        self.assertEqual(consumed,[0,1])

    def test_execution_association_checks_attempt_pose_and_time(self):
        execution=dict(finished=True,map=self.header['map'],manifest_sha256='a'*64,
            attempt_id=self.header['attempt_id'],runs=[dict(run_id='run-a',
            execution=dict(status='arrived',initial_position_cm=[0,0,0],final_position_cm=[0,0,0],sim_s=6))])
        self.assertTrue(self.module.inspect_records(self.records,execution)['finished'])
        for key,value in [('attempt_id','87654321-1234-4234-8234-123456789abc'),
                          ('final_position_cm',[9999,0,0]),('sim_s',.1),('initial_position_cm',[9999,0,0])]:
            changed=copy.deepcopy(execution)
            (changed if key=='attempt_id' else changed['runs'][0]['execution'])[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):
                self.module.inspect_records(self.records,changed)

    def test_attempt_identifier_is_required_and_canonical(self):
        self.header['attempt_id']='not-an-attempt'
        with self.assertRaises(ValueError): self.validate()


if __name__=='__main__':
    unittest.main()
