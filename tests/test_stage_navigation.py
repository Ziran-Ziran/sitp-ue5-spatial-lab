import importlib.util
import json
import os
import subprocess
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/stage_navigation.py'


class StageNavigationTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('stage_navigation', SCRIPT)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.project = self.root / 'local/scenes/synthetic/workspace/UE5/Fixture.uproject'
        self.project.parent.mkdir(parents=True)
        self.project.write_text(json.dumps(dict(FileVersion=3, EngineAssociation='5.8', Description='keep')))
        self.sources = self.root / 'templates/SpatialLab'
        self.sources.mkdir(parents=True)
        (self.sources / 'SpatialLab.Build.cs').write_text('// synthetic')
        (self.sources / 'SpatialLab.cpp').write_text('// synthetic module')

    def test_stage_adds_module_and_targets_preserving_project_metadata(self):
        self.module.stage(self.root, self.project, self.sources)
        data = json.loads(self.project.read_text())
        self.assertEqual(data['Description'], 'keep')
        self.assertEqual(data['Modules'], [dict(Name='SpatialLab', Type='Runtime', LoadingPhase='Default')])
        self.assertTrue((self.project.parent / 'Source/FixtureEditor.Target.cs').exists())
        self.assertTrue((self.project.parent / 'Source/SpatialLab/SpatialLab.cpp').exists())
        self.module.stage(self.root, self.project, self.sources)
        self.assertEqual(len(json.loads(self.project.read_text())['Modules']), 1)

    def test_source_project_outside_experiment_workspace_is_rejected(self):
        project = self.root / 'shared.uproject'
        project.write_text('{}')
        with self.assertRaises(ValueError):
            self.module.stage(self.root, project, self.sources)
        self.assertEqual(project.read_text(), '{}')

    def test_existing_foreign_module_is_preserved(self):
        self.project.write_text(json.dumps(dict(Modules=[dict(Name='Foreign', Type='Runtime')])))
        self.module.stage(self.root, self.project, self.sources)
        self.assertEqual(json.loads(self.project.read_text())['Modules'][0]['Name'], 'Foreign')

    def test_foreign_target_is_not_overwritten(self):
        target = self.project.parent / 'Source/FixtureEditor.Target.cs'
        target.parent.mkdir()
        target.write_text('// existing user work')
        with self.assertRaises(ValueError):
            self.module.stage(self.root, self.project, self.sources)
        self.assertEqual(target.read_text(), '// existing user work')

    def test_agent_configuration_is_fixed_before_world_initialization(self):
        config = self.project.parent / 'Config/DefaultEngine.ini'
        config.parent.mkdir()
        config.write_text('[Existing]\nKeep=True\n')
        self.module.stage(self.root,self.project,self.sources)
        self.module.stage(self.root,self.project,self.sources)
        text=config.read_text()
        self.assertIn('Keep=True',text)
        self.assertEqual(text.count('; BEGIN SITP navigation agent'),1)
        self.assertIn('AgentRadius=34.0,AgentHeight=176.0',text)

    def test_local_native_changes_are_not_overwritten(self):
        self.module.stage(self.root,self.project,self.sources)
        native=self.project.parent/'Source/SpatialLab/SpatialLab.cpp'
        native.write_text('// local experiment change')
        with self.assertRaises(ValueError):
            self.module.stage(self.root,self.project,self.sources)
        self.assertEqual(native.read_text(),'// local experiment change')

    def test_unchanged_generated_copy_can_receive_updated_source(self):
        self.module.stage(self.root,self.project,self.sources)
        (self.sources/'SpatialLab.cpp').write_text('// source update')
        self.module.stage(self.root,self.project,self.sources)
        self.assertEqual((self.project.parent/'Source/SpatialLab/SpatialLab.cpp').read_text(),'// source update')

    def test_linked_source_directory_is_rejected_before_external_write(self):
        outside=self.root/'outside'
        outside.mkdir()
        link=self.project.parent/'Source'
        if os.name=='nt':
            result=subprocess.run(['cmd','/c','mklink','/J',str(link),str(outside)],
                                  capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
            self.assertEqual(result.returncode,0,result.stderr)
        else:
            os.symlink(outside,link,target_is_directory=True)
        self.addCleanup(os.rmdir,link)
        with self.assertRaises(ValueError):
            self.module.stage(self.root,self.project,self.sources)
        self.assertEqual(list(outside.iterdir()),[])


if __name__ == '__main__':
    unittest.main()
