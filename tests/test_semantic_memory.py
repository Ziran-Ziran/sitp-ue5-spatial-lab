"""Synthetic visibility-supervision inputs, never a visual recognition test."""
import importlib.util
from pathlib import Path
import unittest


class SemanticMemoryTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).resolve().parents[1]/'ue5/tools/semantic_memory.py'
        spec=importlib.util.spec_from_file_location('semantic_memory',path)
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def memory(self,ttl=5,run='run-a'):
        return self.module.SemanticMemory(run,ttl,condition='engine_local_visibility_supervision')

    def frame(self,index,time,entities=(),run='run-a'):
        return dict(record_type='frame',run_id=run,frame_id=index,t_sim_s=time,
                    observer_position_cm=[0,0,0],entities=list(entities))

    def entity(self,name='chair-a',position=None):
        return dict(entity_id=name,category='chair',position_cm=position or [100,0,0],visible_sample_count=1)

    def test_hidden_target_is_never_invented_from_an_empty_frame(self):
        memory=self.memory()
        memory.update(self.frame(0,0))
        self.assertIsNone(memory.select('chair'))

    def test_frame_only_drops_occluded_entity_but_temporal_memory_retains_it(self):
        current,self_memory=self.memory(0),self.memory(5)
        for memory in (current,self_memory):
            memory.update(self.frame(0,0,[self.entity()]))
            memory.update(self.frame(1,1))
        self.assertIsNone(current.select('chair'))
        self.assertEqual(self_memory.select('chair')['entity_id'],'chair-a')
        self.assertEqual(self_memory.select('chair')['age_sim_s'],1)

    def test_expiry_uses_simulated_seconds_and_preserves_the_boundary(self):
        memory=self.memory(5)
        memory.update(self.frame(0,0,[self.entity()]))
        memory.update(self.frame(1,5))
        self.assertIsNotNone(memory.select('chair'))
        memory.update(self.frame(2,5.01))
        self.assertIsNone(memory.select('chair'))

    def test_memory_cannot_cross_run_boundaries(self):
        memory=self.memory()
        memory.update(self.frame(0,0,[self.entity()]))
        with self.assertRaises(ValueError):
            memory.update(self.frame(1,1,run='run-b'))
        empty=self.memory(run='run-b')
        empty.update(self.frame(0,0,run='run-b'))
        self.assertIsNone(empty.select('chair'))

    def test_reordered_or_duplicate_frames_fail_without_mutating_memory(self):
        memory=self.memory()
        memory.update(self.frame(0,1,[self.entity()]))
        for frame in (self.frame(1,.9),self.frame(0,2)):
            with self.assertRaises(ValueError):
                memory.update(frame)
        self.assertEqual(memory.select('chair')['age_sim_s'],0)

    def test_selection_ties_are_deterministic(self):
        memory=self.memory()
        memory.update(self.frame(0,0,[self.entity('b'),self.entity('a')]))
        self.assertEqual(memory.select('chair')['entity_id'],'a')

    def test_future_mutation_of_input_does_not_change_saved_observation(self):
        memory=self.memory()
        entity=self.entity()
        memory.update(self.frame(0,0,[entity]))
        entity['position_cm'][0]=9999
        self.assertEqual(memory.select('chair')['position_cm'],[100,0,0])

    def test_engine_supervision_cannot_be_declared_visual_semantics(self):
        with self.assertRaises(ValueError):
            self.module.SemanticMemory('run-a',5,condition='visual_semantics')

    def test_duplicate_entities_and_nonfinite_inputs_fail_atomically(self):
        memory=self.memory()
        for frame in (self.frame(0,0,[self.entity(),self.entity()]),self.frame(0,float('nan'))):
            with self.assertRaises(ValueError):
                memory.update(frame)
        memory.update(self.frame(0,0,[self.entity()]))
        self.assertIsNotNone(memory.select('chair'))


if __name__=='__main__':
    unittest.main()
