load("//tools/workspace:github.bzl", "github_archive")

def libzippp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "ctabin/libzippp",
        commit = "libzippp-v7.1-1.10.1",
        sha256 = "9ded3c4b5641e65d2b3a3dd0cbc4106209ee17c17df70e5187e7171420752546",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
