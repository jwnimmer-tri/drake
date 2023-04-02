load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def spdlog_internal_repository(
        name,
        mirrors = None):
    # Here, we elect to use the same version as Ubuntu 20.04, even
    # though it is not the newest revision.  Sticking with a single,
    # older revision helps reduce spurious CI failures.
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        commit = "v1.11.0",
        sha256 = "ca5cae8d6cac15dae0ec63b21d6ad3530070650f68076f3a4a862ca293a858bb",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
