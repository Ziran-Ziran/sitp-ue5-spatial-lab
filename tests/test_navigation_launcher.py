import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


class NavigationLauncherTests(unittest.TestCase):
    def setUp(self):
        path=Path(__file__).resolve().parents[1]/'scripts/navigation_experiment.py'
        spec=importlib.util.spec_from_file_location('navigation_experiment',path)
        self.module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)

    def test_results_cannot_overwrite_an_existing_experiment(self):
        result=self.root/'output/fixture'
        self.module.create_run_directory(self.root,result)
        (result/'evidence.txt').write_text('keep')
        with self.assertRaises(FileExistsError):
            self.module.create_run_directory(self.root,result)
        self.assertEqual((result/'evidence.txt').read_text(),'keep')

    def test_output_cannot_write_to_source_or_a_shared_directory(self):
        for path in (self.root/'ue5/tools/oops',self.root.parent/'shared-output'):
            with self.assertRaises(ValueError):
                self.module.create_run_directory(self.root,path)
            self.assertFalse(path.exists())

    def test_unsupported_engine_fails_before_any_process_or_staging(self):
        engine=self.root/'engine'
        version=engine/'Engine/Build/Build.version'
        version.parent.mkdir(parents=True)
        version.write_text(json.dumps(dict(MajorVersion=5,MinorVersion=7)))
        with self.assertRaises(ValueError):
            self.module.engine_tools(engine)
