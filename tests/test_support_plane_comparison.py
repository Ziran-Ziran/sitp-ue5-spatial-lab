from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from compare_support_plane_transfer import verify_producer,paired_summary


class SupportPlaneProducerTests(unittest.TestCase):
    def setUp(self):
        self.directory=Path('output/example').absolute();self.project=Path('local/Scene.uproject').absolute()
        self.bundle=dict(map='/Game/SpatialLab/City')
        self.command=['ue',str(self.project),self.bundle['map'],'-game',
            '-SITPDepthPlan='+str(self.directory/'plan.json').replace('\\','/'),
            '-SITPResult='+str(self.directory/'execution.json').replace('\\','/'),
            '-benchmark','-UseFixedTimeStep','-FPS=60','-deterministic']

    def verify(self,command):return verify_producer(command,self.project,self.bundle,self.directory)

    def test_exact_project_map_plan_output_and_fixed_time_producer_passes(self):self.verify(self.command)

    def test_unused_expected_map_project_or_duplicate_flags_are_rejected(self):
        for flag in ('-sitpdepthplan=elsewhere.json','-SITPResult=elsewhere.json','-fps=30','-game'):
            with self.subTest(flag=flag),self.assertRaises(ValueError):self.verify(self.command+[flag])
        for position,replacement in ((1,str(Path('Other.uproject').absolute())),(2,'/Game/SpatialLab/Other')):
            command=list(self.command);command.append(command[position]);command[position]=replacement
            with self.subTest(position=position),self.assertRaises(ValueError):self.verify(command)

    def test_missing_fixed_time_flag_cannot_claim_this_protocol(self):
        with self.assertRaises(ValueError):self.verify([t for t in self.command if t!='-UseFixedTimeStep'])

    def test_identical_depth_frames_expose_correlated_improvements(self):
        from support_plane_execution import MODELS
        runs=[];scores=[];hashes={}
        for sequence in ('sequence_000','sequence_001'):
            for model in MODELS:
                key=sequence+'-'+model;runs.append(dict(sequence=sequence,frame_id=1,mode=model,run_id=key))
                scores.append(dict(run_id=key,success=model==MODELS[1],status='arrived' if model==MODELS[1] else 'not_executed'))
            for view in ('level','down45','nadir'):hashes[f'{sequence}/{view}/depth/frame_000001.png']='a'*64
        paired=paired_summary(dict(runs=runs,source_files_sha256=hashes),dict(runs=scores))
        self.assertEqual(paired['paired_frames'],2);self.assertEqual(paired['transitions'],dict(plane_only_arrived=2))
        self.assertEqual(paired['unique_depth_input_groups'],1);self.assertEqual(paired['changed_unique_depth_input_groups'],1)
