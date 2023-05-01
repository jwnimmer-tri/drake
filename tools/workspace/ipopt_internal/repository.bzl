# -*- python -*-

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def ipopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Ipopt",
        commit = "releases/3.14.10",
        sha256 = "b73d705ca05a8fb47392ca7e31c4da81ae7d0eb751767cd04ba2bb19b7f140f9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
