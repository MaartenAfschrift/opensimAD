"""Regression tests for IDE environments whose PATH omits CMake."""
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from opensimad import build


class CMakeDiscoveryTests(unittest.TestCase):
    def test_path_has_priority_over_standard_installation(self):
        with patch.object(build.shutil, 'which', return_value='custom/cmake.exe'):
            self.assertEqual(build.find_cmake({'PATH': 'custom'}), 'custom/cmake.exe')

    def test_explicit_override_and_invalid_override(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / 'custom cmake.exe'
            executable.touch()
            self.assertEqual(build.find_cmake({'OPENSIMAD_CMAKE': str(executable)}), str(executable.resolve()))
            with self.assertRaisesRegex(FileNotFoundError, 'OPENSIMAD_CMAKE'):
                build.find_cmake({'OPENSIMAD_CMAKE': str(executable.parent / 'missing.exe')})

    @unittest.skipUnless(os.name == 'nt', 'Windows installation fallback')
    def test_standard_installation_without_path(self):
        with tempfile.TemporaryDirectory() as directory:
            executable = Path(directory) / 'CMake' / 'bin' / 'cmake.exe'
            executable.parent.mkdir(parents=True)
            executable.touch()
            environment = {'PATH': '', 'ProgramW6432': directory}
            with patch.object(build.shutil, 'which', return_value=None):
                self.assertEqual(build.find_cmake(environment), str(executable))

    def test_missing_cmake_reports_remedy_and_stage(self):
        with patch.object(build.shutil, 'which', return_value=None), patch.object(Path, 'is_file', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'configuration.*failed to start') as caught:
                build.run(['cmake', '--version'], 'configuration', Path.cwd(), env={'PATH': ''})
            self.assertIn('OPENSIMAD_CMAKE', str(caught.exception))


if __name__ == '__main__':
    unittest.main()
