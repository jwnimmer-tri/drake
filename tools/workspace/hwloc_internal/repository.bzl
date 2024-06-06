load("//tools/workspace:github.bzl", "github_archive")

def hwloc_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "open-mpi/hwloc",
        commit = "hwloc-2.10.0",
        sha256 = "9c5279b16b84c30e789b630568a62e9787d081f6b1932c9010f1e6db2b058489",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
