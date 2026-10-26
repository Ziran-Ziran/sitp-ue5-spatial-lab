import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from visual_marker_tasks import build_tasks,validate_tasks


class VisualMarkerTaskTests(unittest.TestCase):
    def test_complete_fixed_tasks_do_not_contain_target_catalog_or_coordinates(self):
        tasks=build_tasks('/Game/SpatialLab/Markers','a'*64)
        self.assertEqual(len(tasks['runs']),12)
        self.assertEqual({r['category'] for r in tasks['runs']},{'red_marker','blue_marker'})
        for row in tasks['runs']:
            self.assertNotIn('goal_cm',row);self.assertNotIn('entities',row)
        self.assertEqual(tasks['pose_source'],'engine_ground_truth_reset_and_actuation_only')

    def test_removed_failures_and_changed_task_fields_are_rejected(self):
        original=build_tasks('/Game/SpatialLab/Markers','a'*64)
        for mutate in (lambda t:t['runs'].pop(),lambda t:t['runs'][0].update(goal_cm=[0,0,0])):
            tasks=copy.deepcopy(original);mutate(tasks)
            with self.assertRaises(ValueError):validate_tasks(tasks)
