load("//tools/workspace:github.bzl", "github_archive")

def coinutils_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/CoinUtils",
        commit = "releases/2.11.12",
        sha256 = "eef1785d78639b228ae2de26b334129fe6a7d399c4ac6f8fc5bb9054ba00de64",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            # XXX probably rm me
            # ":patches/no_iostream.patch",
        ],
        patch_cmds = [
            # XXX document me
            "sed -i -e 's|<iostream>|<drake_std_cout_cerr.h>|;' $(find CoinUtils/src -name '*.[ch]*')",  # noqa
        ],
        mirrors = mirrors,
    )
