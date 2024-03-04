"""A wrapper script that calls the platform-specific ktx binary with the
appropriate library paths.
"""

import os
from pathlib import Path
import platform
import sys

from bazel_tools.tools.python.runfiles import runfiles


def main():
    machine = platform.machine()
    bazel_dir = Path(runfiles.Create().EnvVars()["RUNFILES_DIR"])
    base_dir, = bazel_dir.glob(f"ktx_software_internal/*-{machine}")
    exe = base_dir / "bin/ktx"
    os.execv(exe, [exe] + sys.argv[1:])


assert __name__ == "__main__"
main()
