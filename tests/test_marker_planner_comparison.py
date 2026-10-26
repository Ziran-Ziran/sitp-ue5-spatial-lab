import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from compare_visual_marker_runs import compare,unique_argument
from continuous_grid_planner import VARIANTS


class MarkerPlannerComparisonTests(unittest.TestCase):
    def rows(self):
        return [dict(variant=v,report=dict(overall=dict(runs=12,successes=8,collisions=0)),truth_entities=[dict(category='red_marker')],
            fixture_source_sha256='a'*64,native_binary_sha256='b'*64) for v in VARIANTS]

    def test_complete_comparison_retains_each_denominator(self):
        report=compare(self.rows());self.assertEqual([r['runs'] for r in report['variants']],[12,12,12])

    def test_missing_duplicate_or_world_change_rejected(self):
        rows=self.rows()
        for changed in (rows[:2],[rows[0],rows[0],rows[2]]):
            with self.assertRaises(ValueError):compare(changed)
        for key in ('truth_entities','fixture_source_sha256','native_binary_sha256'):
            changed=copy.deepcopy(rows);changed[1][key]='changed'
            with self.assertRaises(ValueError):compare(changed)

    def test_case_insensitive_duplicate_producer_flags_rejected(self):
        unique_argument(['-game','-SITPTools=correct'],'-SITPTools=correct')
        for command in (['-SITPTools=wrong'],['-SITPTools=correct','-sitptools=wrong']):
            with self.assertRaises(ValueError):unique_argument(command,'-SITPTools=correct')
