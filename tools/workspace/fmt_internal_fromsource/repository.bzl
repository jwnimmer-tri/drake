load("//tools/workspace:github.bzl", "github_archive")

def fmt_internal_fromsource_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "fmtlib/fmt",
        # Here, we elect to use the same version as Ubuntu 22.04, even though
        # it is not the newest revision. Sticking with a single, older revision
        # helps reduce spurious CI failures.
        default = "8.1.1",
        commit_pin = True,
        sha256 = "3d794d3cf67633b34b2771eb9f073bde87e846e0d395d254df7b211ef1ec7346",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
