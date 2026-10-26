from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from compare_online_query_runs import compare
from query_grid_index import POLICIES


class OnlineQueryComparisonTests(unittest.TestCase):
    def rows(self):
        return [dict(policy=p,native_binary_sha256='a'*64,canonical_task_sha256='b'*64,
            report=dict(overall=dict(runs=96,successes=12),by_mode={},runs=[]),raw_execution_sha256='c'*64) for p in POLICIES]

    def test_both_predeclared_conditions_and_denominators_retained(self):
        result=compare(self.rows());self.assertEqual(len(result['conditions']),2)
        self.assertEqual([r['overall']['runs'] for r in result['conditions']],[96,96])

    def test_missing_duplicate_or_changed_tasks_build_rejected(self):
        rows=self.rows()
        for bad in (rows[:1],[rows[0],rows[0]]):
            with self.assertRaises(ValueError):compare(bad)
        for key in ('native_binary_sha256','canonical_task_sha256'):
            rows=self.rows();rows[1][key]='changed'
            with self.assertRaises(ValueError):compare(rows)
