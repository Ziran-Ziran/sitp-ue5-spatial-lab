from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from support_physics_goal_report import surface_proximity,summarize


class SupportGoalDiagnosticsTests(unittest.TestCase):
    def test_known_construction_plane_explains_height_contract_without_changing_it(self):
        metrics=surface_proximity('up10_boundary',[101.6273971294349,6002.49442740164,20.594158009589066])
        self.assertAlmostEqual(metrics['constructed_ground_goal_cm'][2],17.632698070846498)
        self.assertTrue(metrics['endpoint_near_constructed_ground_goal'])
        self.assertFalse(metrics['endpoint_near_original_flat_goal'])
        self.assertEqual(metrics['xy_tolerance_cm'],10);self.assertEqual(metrics['z_tolerance_cm'],20)

    def test_flat_nearby_wall_blocked_or_unmeasured_does_not_become_navigation_success(self):
        self.assertTrue(surface_proximity('flat',[100,0,2])['endpoint_near_constructed_ground_goal'])
        self.assertFalse(surface_proximity('low170',[30,14000,2])['endpoint_near_constructed_ground_goal'])
        rows=[dict(native_success=False,native_failure_reason='endpoint_height_mismatch',measurement_valid=True,
                proximity=surface_proximity('up10_boundary',[100,6000,21])),
            dict(native_success=False,native_failure_reason='not_executed_planning_start_clearance_blocked',measurement_valid=False,proximity=None)]
        summary=summarize(rows)
        self.assertEqual(summary['native_contract_successes'],0)
        self.assertEqual(summary['posthoc_endpoints_near_constructed_ground'],1)
        self.assertEqual(summary['unmeasured_rows'],1);self.assertEqual(summary['full_rows'],2)
        self.assertEqual(summary['native_failure_counts']['endpoint_height_mismatch'],1)

    def test_missing_case_nonfinite_or_changed_truth_is_rejected(self):
        for case,point in [('unknown',[0,0,0]),('up10_boundary',[0,float('nan'),0]),('flat',[0,0])]:
            with self.assertRaises(ValueError):surface_proximity(case,point)


if __name__=='__main__':unittest.main()
