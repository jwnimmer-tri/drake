"""Drake's wrapper for the clang-format binary.
"""

from pathlib import Path


def get_clang_format_path():
    path = Path("external/llvm-project/clang/clang-format")
    if not path.is_file():
        raise RuntimeError(f"Could not find required clang-format at {path}")
    return path
