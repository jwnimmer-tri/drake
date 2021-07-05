# -*- python -*-

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

def eigen_repository(name):
    new_git_repository(
        name = name,
        remote = "https://gitlab.com/libeigen/eigen.git",
        commit = "4780d8dfb2b0e9bcff063c80f4ffce71d9d7a725",
        shallow_since = "1624302437 +0000",
        build_file_content = """
cc_library(
    name = "eigen",
    hdrs = glob(["Eigen/**", "unsupported/Eigen/**"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)
""",
    )
