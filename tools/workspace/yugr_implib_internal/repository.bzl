load("@drake//tools/workspace:github.bzl", "github_archive")

def yugr_implib_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "yugr/Implib.so",
        commit = "5fb84c2a750434b9df1da67d67b749eb929598f1",
        sha256 = "10de0a616df24849f2a883747784c115f209708960e44556f5ce384de6f103e8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
