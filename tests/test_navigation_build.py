import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


class NavigationBuildTests(unittest.TestCase):
    def setUp(self):
        spec=importlib.util.spec_from_file_location('navigation_build',Path(__file__).resolve().parents[1]/'scripts/navigation_build.py')
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        source=self.root/'ue5/tools/native/SpatialLab'
        source.mkdir(parents=True)
        self.source=source/'SpatialLab.cpp'
        self.source.write_text('// fixture implementation')
        self.project=self.root/'local/scenes/fixture/workspace/UE5/Fixture.uproject'
        self.project.parent.mkdir(parents=True)
        self.project.write_text('{}')
        self.staged=self.project.parent/'Source/SpatialLab/SpatialLab.cpp'
        self.staged.parent.mkdir(parents=True)
        self.staged.write_bytes(self.source.read_bytes())
        (self.project.parent/'.sitp-navigation-stage.json').write_text(json.dumps(dict(
            files={'Source/SpatialLab/SpatialLab.cpp':self.module.file_hash(self.staged)})))
        self.dll=self.project.parent/'Binaries/Win64/UnrealEditor-SpatialLab.dll'
        self.dll.parent.mkdir(parents=True)
        self.dll.write_bytes(b'synthetic binary fixture, never executed')

    def test_record_binds_current_source_to_actual_binary(self):
        record=self.module.record_build(self.root,self.project)
        self.assertEqual(self.module.verify_build(self.root,self.project),record)

    def test_source_change_requires_build_before_run(self):
        self.module.record_build(self.root,self.project)
        self.source.write_text('// new uncompiled implementation')
        with self.assertRaises(ValueError):
            self.module.verify_build(self.root,self.project)

    def test_binary_replacement_or_missing_record_is_rejected(self):
        with self.assertRaises(ValueError):
            self.module.verify_build(self.root,self.project)
        self.module.record_build(self.root,self.project)
        self.dll.write_bytes(b'other binary')
        with self.assertRaises(ValueError):
            self.module.verify_build(self.root,self.project)

    def test_build_record_cannot_bind_different_staged_and_public_sources(self):
        self.staged.write_text('// changed while compiling')
        with self.assertRaises(ValueError):
            self.module.record_build(self.root,self.project)
