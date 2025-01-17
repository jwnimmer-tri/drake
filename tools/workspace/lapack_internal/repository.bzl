load("//tools/workspace:github.bzl", "github_archive")

def lapack_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Reference-LAPACK/lapack",
        commit = "v3.12.0",
        # 3.12.1 requires BLAS Level 3, which we don't have yet.
        commit_pin = True,
        sha256 = "eac9570f8e0ad6f30ce4b963f4f033f0f643e7c3912fc9ee6cd99120675ad48b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
