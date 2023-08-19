"""Drake's wrapper for the clang-format binary.
"""

from pathlib import Path

from python import runfiles


def get_clang_format_path():
    manifest = runfiles.Create()
    path = Path(manifest.Rlocation("llvm-project/clang/clang-format"))
    if not path.is_file():
        raise RuntimeError(f"Could not find required clang-format at {path}")
    return path
