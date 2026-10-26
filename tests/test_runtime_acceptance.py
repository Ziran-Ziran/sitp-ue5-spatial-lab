import importlib.util
from pathlib import Path
import unittest


class RuntimeAcceptanceTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).resolve().parents[1]/'ue5/tools/runtime_acceptance.py'
        spec=importlib.util.spec_from_file_location('runtime_acceptance',path)
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.report=dict(finished=True,expected_runs=4,units='centimeters',position_reference='capsule_feet',
                         evidence_kind='ue_game_world_character_movement',runs=[
            dict(run_id='detour',execution=dict(status='arrived',continuous_path_length_cm=1900,
                final_position_cm=[690,0,2],collisions=0,sim_s=6.5)),
            dict(run_id='spawn-blocked',execution=dict(status='spawn_not_on_navmesh',collisions=None)),
            dict(run_id='unreachable',execution=dict(status='runtime_path_rejected',collisions=0)),
            dict(run_id='direct-blocked',execution=dict(status='movement_blocked',collisions=20,
                final_position_cm=[-80,0,2]))])

    def test_expected_negative_cases_are_acceptance_not_task_successes(self):
        result=self.module.validate_synthetic(self.report)
        self.assertTrue(result['passed'])
        self.assertEqual(result['expected_failures'],3)
        self.assertEqual(result['evidence_kind'],'synthetic_ue_game_world_integration')

    def test_no_collision_observation_cannot_pass_wall_acceptance(self):
        self.report['runs'][3]['execution']['collisions']=None
        self.assertFalse(self.module.validate_synthetic(self.report)['passed'])

    def test_exit_zero_without_all_finished_runs_cannot_pass(self):
        self.report['finished']=False
        self.assertFalse(self.module.validate_synthetic(self.report)['passed'])
        self.report['finished']=True
        self.report['runs'].pop()
        self.assertFalse(self.module.validate_synthetic(self.report)['passed'])

    def test_missing_detour_or_wall_traversal_cannot_pass(self):
        self.report['runs'][0]['execution']['continuous_path_length_cm']=1400
        self.assertFalse(self.module.validate_synthetic(self.report)['passed'])


if __name__=='__main__':
    unittest.main()
