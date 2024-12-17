load("//tools/workspace:github.bzl", "github_archive")

def ipopt_internal_fromsource_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Ipopt",
        upgrade_advice = """
        When updating, the ipopt version in
        tools/wheel/image/dependencies/projects.cmake
        must also be updated.
        """,
        commit = "releases/3.14.16",
        sha256 = "cc8c217991240db7eb14189eee0dff88f20a89bac11958b48625fa512fe8d104",  # noqa
        build_file = ":package.BUILD.bazel",
        patch_cmds = [
            # XXX document me
            "sed -i -e 's|<iostream>|<drake_std_cout_cerr.h>|;' $(find src -name '*.[ch]*')",  # noqa
        ],
        mirrors = mirrors,
    )
