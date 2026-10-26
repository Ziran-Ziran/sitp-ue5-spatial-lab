import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from compare_marker_horizon_runs import compare
from marker_goal_horizon import POLICIES


class MarkerHorizonComparisonTests(unittest.TestCase):
    def rows(self):
        return [dict(policy=policy,canonical_manifest_sha256='a'*64,native_binary_sha256='b'*64,scene_sha256='c'*64,
            report=dict(overall=dict(runs=96,successes=0))) for policy in POLICIES]

    def test_complete_pair_preserves_all_fixed_denominators(self):
        result=compare(self.rows());self.assertEqual([row['report']['overall']['runs'] for row in result['conditions']],[96,96])

    def test_duplicate_policy_missing_cases_and_changed_native_scene_or_tasks_rejected(self):
        rows=self.rows()
        for changed in (rows[:1],[rows[0],rows[0]]):
            with self.assertRaises(ValueError):compare(changed)
        for key in ('canonical_manifest_sha256','native_binary_sha256','scene_sha256'):
            changed=copy.deepcopy(rows);changed[1][key]='different'
            with self.assertRaises(ValueError):compare(changed)
