import os
import subprocess
import unittest

import install_test_helper


class TestMypyInstall(unittest.TestCase):
    """Tests that pydrake's `*.pyi` files are operating correctly."""

    def setUp(self):
        self._python = install_test_helper.get_python_executable()

        # Override PYTHONPATH and MYPYPATH to only use the installed `pydrake`.
        install_dir = install_test_helper.get_install_dir()
        py_dir = install_test_helper.get_python_site_packages_dir(install_dir)
        self._env = dict(os.environ)
        self._env["PYTHONPATH"] = py_dir
        self._env["MYPYPATH"] = py_dir
        self._cwd = os.environ["TEST_TMPDIR"]

    def test_mypy(self):
        proc = subprocess.run(
            [self._python, "-m", "mypy", "-m", "pydrake"],
            env=self._env, cwd=self._cwd)
        self.assertEqual(proc.returncode, 1)



if __name__ == '__main__':
    unittest.main()
