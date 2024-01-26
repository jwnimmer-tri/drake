load("//tools/workspace:github.bzl", "github_archive")

def spdlog_internal_fromsource_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gabime/spdlog",
        # Here, we elect to use the same version as Ubuntu 22.04, even though
        # it is not the newest revision. Sticking with a single, older revision
        # helps reduce spurious CI failures.
        default = "v1.9.2",
        commit_pin = True,
        sha256 = "6fff9215f5cb81760be4cc16d033526d1080427d236e86d70bb02994f85e3d38",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
