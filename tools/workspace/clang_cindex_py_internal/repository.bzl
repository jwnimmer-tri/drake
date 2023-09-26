load("//tools/workspace:github.bzl", "github_archive")

def clang_cindex_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "llvm/llvm-project",
        commit = "llvmorg-12.0.1",
        attachment = "clang-12.0.1.src.tar.xz",
        extra_strip_prefix = "clang-12.0.1.src/bindings/python",
        sha256 = "6e912133bcf56e9cfe6a346fa7e5c52c2cde3e4e48b7a6cc6fcc7c75047da45f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
