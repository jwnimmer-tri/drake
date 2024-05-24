load("//tools/workspace:github.bzl", "github_archive")

def spral_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ralna/spral",
        commit = "v2024.05.08",
        sha256 = "0795c10c1c4dab1cf8c2de4024296d75d9d83b7525e82c77584c16060e29e4f5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
