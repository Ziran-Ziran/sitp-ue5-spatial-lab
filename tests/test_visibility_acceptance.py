import copy
import unittest
import test_semantic_observations as fixture_module
from visibility_acceptance import validate_synthetic_visibility


class VisibilityAcceptanceTests(unittest.TestCase):
    def setUp(self):
        fixture=fixture_module.ObservationStreamTests()
        fixture.setUp()
        self.records=copy.deepcopy(fixture.records)
        for row in self.records:
            if row.get('run_id')=='run-a': row['run_id']='detour'
        self.records[4]['entities']=[dict(entity_id='hidden-lamp',category='lamp',
            position_cm=[100,0,0],visible_sample_count=1)]
        self.fixture=dict(map=fixture.header['map'],semantic_controls=dict(
            visible_chair='observed-chair',occluded_lamp='hidden-lamp'))

    def test_visible_and_occluded_then_revealed_controls_pass(self):
        self.assertTrue(validate_synthetic_visibility(self.fixture,self.records)['passed'])

    def test_category_inversion_is_not_accepted(self):
        self.records[2]['entities'][0]['category']='lamp'
        self.records[4]['entities'][0]['category']='chair'
        self.assertFalse(validate_synthetic_visibility(self.fixture,self.records)['passed'])

    def test_initial_occlusion_failure_is_not_accepted(self):
        self.records[2]['entities'].extend(self.records[4]['entities'])
        self.assertFalse(validate_synthetic_visibility(self.fixture,self.records)['passed'])

    def test_never_revealed_target_is_not_accepted(self):
        self.records[4]['entities']=[]
        self.assertFalse(validate_synthetic_visibility(self.fixture,self.records)['passed'])
