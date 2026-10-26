import json
from pathlib import Path
import sys
import tempfile
import unittest
import test_semantic_observations as fixtures

TOOLS=Path(__file__).resolve().parents[1]/'ue5/tools'
sys.path.insert(0,str(TOOLS))
from replay_source_binding import binding,sha256


class ReplaySourceBindingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.project=self.root/'scene/Project.uproject'
        self.map=self.root/'scene/Content/SpatialLab/Fixture.umap';self.map.parent.mkdir(parents=True)
        self.map.write_bytes(b'synthetic-map')
        fixture=fixtures.ObservationStreamTests();fixture.setUp()
        fixture.records[0]['route_source']='observation_dependent_engine_navmesh_geometry'
        self.observations=self.root/'observations.jsonl'
        self.observations.write_text(''.join(json.dumps(row)+'\n' for row in fixture.records),encoding='utf-8')
        self.execution=self.root/'execution.json'
        state=dict(status='arrived',initial_position_cm=[0,0,0],final_position_cm=[0,0,0],sim_s=6)
        execution=dict(finished=True,map=fixture.header['map'],manifest_sha256=fixture.header['manifest_sha256'],
            attempt_id=fixture.header['attempt_id'],evidence_kind='ue_game_world_observation_dependent_search',
            runs=[dict(run_id='run-a',execution=state)])
        self.execution.write_text(json.dumps(execution),encoding='utf-8')
        self.spec=self.root/'source.job.json'
        command=['ue',str(self.project),'-SITPSearch=missions.json','-SITPObservations='+str(self.observations),
                 '-SITPResult='+str(self.execution)]
        self.spec.write_text(json.dumps(dict(command=command,input_sha256={str(self.map):sha256(self.map)})),encoding='utf-8')
        self.spec.with_suffix('.status.json').write_text(json.dumps(dict(command=command,exit_code=0,
            timed_out=False,tree_cleaned=True,inputs_verified=True,spec_sha256=sha256(self.spec),
            verified_input_sha256={str(self.map):sha256(self.map)})),encoding='utf-8')

    def test_original_completed_attempt_and_map_bind(self):
        result=binding(self.spec,self.observations,project=self.project)
        self.assertEqual(result['map_sha256'],sha256(self.map))
        self.assertEqual(result['execution_sha256'],sha256(self.execution))

    def test_same_package_with_changed_map_bytes_is_rejected(self):
        self.map.write_bytes(b'changed-scene')
        with self.assertRaises(ValueError):binding(self.spec,self.observations)

    def test_rehashed_spec_cannot_reuse_original_completed_status(self):
        self.map.write_bytes(b'changed-scene')
        data=json.loads(self.spec.read_text());data['input_sha256'][str(self.map)]=sha256(self.map)
        self.spec.write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaises(ValueError):binding(self.spec,self.observations)

    def test_legacy_status_without_snapshot_is_rejected(self):
        path=self.spec.with_suffix('.status.json');data=json.loads(path.read_text())
        del data['spec_sha256'];del data['verified_input_sha256']
        path.write_text(json.dumps(data),encoding='utf-8')
        with self.assertRaises(ValueError):binding(self.spec,self.observations)

    def test_other_observation_file_or_project_is_rejected(self):
        other=self.root/'other.jsonl';other.write_bytes(self.observations.read_bytes())
        with self.assertRaises(ValueError):binding(self.spec,other)
        with self.assertRaises(ValueError):binding(self.spec,self.observations,project=self.root/'other.uproject')

    def test_mixed_attempt_execution_is_rejected(self):
        value=json.loads(self.execution.read_text());value['attempt_id']='87654321-1234-4234-8234-123456789abc'
        self.execution.write_text(json.dumps(value),encoding='utf-8')
        with self.assertRaises(ValueError):binding(self.spec,self.observations)
