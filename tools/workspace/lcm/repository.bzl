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
        commit = "050ce9ee512fe8a42ea26934bdb80f64200ef057",
        sha256 = "410d0dc4d31a8153aab2b2e28a879ae5719556e93a7577365a8f2ca6d0f95bab",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/copts.patch",
            ":patches/drake_shared_library.patch",
            ":patches/install.patch",
            ":patches/respell_bzlmod_deps.patch",
        ],
        mirrors = mirrors,
    )
