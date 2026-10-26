import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ue5/tools'))
from online_depth_tasks import build_tasks,validate_tasks


class OnlineDepthTasksTests(unittest.TestCase):
    def setUp(self):
        self.frames=[dict(sequence='sequence_000',source_run_id='search',frame_id=i,source_frame_id=i*10,t_source_sim_s=i,
            start_cm=[100,200,0],heading_deg=90) for i in range(4)]

    def test_all_source_frames_enter_both_conditions_without_filtering(self):
        tasks=build_tasks('/Game/SpatialLab/City','a'*64,self.frames,exposure=13,source_binding={'capture_sha256':'b'*64})
        self.assertEqual(len(tasks['runs']),8)
        self.assertEqual(tasks['runs'][0]['goal_cm'],[100,500,0])
        self.assertEqual({r['mode'] for r in tasks['runs']},{'initial_sweep_once','receding_sweep'})
        bad=copy.deepcopy(tasks);bad['runs'].pop()
        with self.assertRaises(ValueError):validate_tasks(bad)

    def test_no_catalog_or_engine_geometry_is_added_to_a_task(self):
        tasks=build_tasks('/Game/SpatialLab/City','a'*64,self.frames,exposure=13,source_binding={'capture_sha256':'b'*64})
        self.assertNotIn('entities',str(tasks));self.assertEqual(tasks['geometry_source'],'current_stopped_rgbd_depth_only')

    def test_versioned_query_policy_keeps_all_frames_and_binds_every_run(self):
        for policy in ('legacy_floor','boundary_snap_1e-9_cell'):
            tasks=build_tasks('/Game/SpatialLab/City','a'*64,self.frames,exposure=13,source_binding={},query_cell_policy=policy)
            self.assertEqual(tasks['schema_version'],2);self.assertEqual(len(tasks['runs']),8)
            self.assertEqual({r['query_cell_policy'] for r in tasks['runs']},{policy})
            tasks['runs'][0]['query_cell_policy']='invented'
            from online_depth_tasks import digest
            tasks['manifest_sha256']=digest(tasks)
            with self.assertRaises(ValueError):validate_tasks(tasks)
