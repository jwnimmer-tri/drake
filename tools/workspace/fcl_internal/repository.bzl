load("//tools/workspace:github.bzl", "github_archive")

def fcl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "e5efcc41b57b2d0da3bf183480f1298a6d531f44",
        sha256 = "37aa84608083170329b6d9f9b07dc20d813b84d85546d1e3f1417cc8c2583c6e",  # noqa
        build_file = ":package.BUILD.bazel",
        patch_cmds = [
            # XXX document me
            "sed -i -e 's|<iostream>|<drake_cout_cerr.h>|; s| std::cout | drake::cout |g; s| std::cerr | drake::cerr |g;' $(find include src -name '*.[ch]*')",  # noqa
        ],
        mirrors = mirrors,
    )
