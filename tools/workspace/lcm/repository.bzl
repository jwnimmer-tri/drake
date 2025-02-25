load("//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        upgrade_advice = """
        When updating, lcm needs its own pull request separate from the rest of
        the monthly upgrades.
        """,
        # TOOD(jwnimmer-tri) Once LCM has its next tagged release >v1.5.1, we
        # should switch this back to a release tag instead of this hash.
        commit = "e4686e6670eb675bf9305b5f7cc033d741a4ccf8",
        sha256 = "fe669dd8938ea600549349df4c6b92382d30fc7426b324602a991f19ac8fad38",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/copts.patch",
            ":patches/drake_shared_library.patch",
            ":patches/install.patch",
            ":patches/respell_bzlmod_deps.patch",
        ],
        mirrors = mirrors,
    )
