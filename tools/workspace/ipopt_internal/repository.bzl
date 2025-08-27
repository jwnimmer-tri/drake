load("//tools/workspace:github.bzl", "github_archive")

def ipopt_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Ipopt",
        commit = "releases/3.14.19",
        sha256 = "b3eb84a23812b53a3325bcd2c599de2b0f5df45a18ed251f9e3c1cd893136287",  # noqa
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
