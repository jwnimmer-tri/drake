from pathlib import Path

from clang import cindex
from python import runfiles


def get_include_flags():
    """....
    """
    manifest = runfiles.Create()
    path = Path(manifest.Rlocation("llvm/lib/libclang.so.19.1.3"))
    cindex.Config.set_library_file(path)

    libcxx_v1_dir = Path(manifest.Rlocation(
        "llvm/include/c++/v1/optional")).parent
    llvm_dir = libcxx_v1_dir.parent.parent.parent
    libcxx_x86_dir = llvm_dir / "include/x86_64-unknown-linux-gnu/c++/v1"
    libcxx_libinclude_dir = llvm_dir / "lib/clang/19.1.3/include"
    result = [
        "-nostdinc",
        # "-nostdlibinc",
        f"-I{libcxx_v1_dir}",
        f"-I{libcxx_x86_dir}",
        f"-I{libcxx_libinclude_dir}",
        "-I/usr/include",
        "-I/usr/include/x86_64-linux-gnu",
        "-I/usr/lib/gcc/x86_64-linux-gnu/11/include",
    ]
    return result
