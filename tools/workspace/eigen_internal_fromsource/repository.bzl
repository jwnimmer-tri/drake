load("//tools/workspace:github.bzl", "github_archive")

def eigen_internal_fromsource_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "eigen-mirror/eigen",
        commit = "3.4.0",
        sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
