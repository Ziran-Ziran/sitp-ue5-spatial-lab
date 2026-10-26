from pathlib import Path
import subprocess
import sys
import unittest


class CityMarkerFixtureImportTests(unittest.TestCase):
    def test_commandlet_style_import_finds_local_helper_without_numeric_dependencies(self):
        script=Path(__file__).resolve().parents[1]/'ue5/tools/city_marker_fixture.py'
        code="import runpy,sys,types;sys.modules['unreal']=types.ModuleType('unreal');runpy.run_path(sys.argv[1],run_name='import_contract_probe')"
        result=subprocess.run([sys.executable,'-I','-c',code,str(script)],capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)
