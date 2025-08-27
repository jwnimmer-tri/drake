load("//tools/workspace:github.bzl", "github_archive")

def ipopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Ipopt",
        commit = "releases/3.14.17",
        sha256 = "17ab8e9a6059ab11172c184e5947e7a7dda9fed0764764779c27e5b8e46f3d75",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/upstream/atomic.patch",
            ":patches/upstream/iostream_includes.patch",
        ],
        patch_cmds = [
            # XXX document me
            "sed -i -e 's|<iostream>|<drake_std_cout_cerr.h>|;' $(find src -name '*.[ch]*')",  # noqa
        ],
        mirrors = mirrors,
    )
