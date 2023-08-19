# This file marks a workspace root for the Bazel build system.
# See `https://bazel.build/`.

workspace(name = "drake")

load("//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()

load("@build_bazel_apple_support//crosstool:setup.bzl", "apple_cc_configure")

apple_cc_configure()

# Add some special heuristic logic for using CLion with Drake.
load("//tools/clion:repository.bzl", "drake_clion_environment")

drake_clion_environment()

load("@bazel_skylib//lib:versions.bzl", "versions")

# This needs to be in WORKSPACE or a repository rule for native.bazel_version
# to actually be defined. The minimum_bazel_version value should match the
# version passed to the find_package(Bazel) call in the root CMakeLists.txt.
versions.check(minimum_bazel_version = "6.0")

# The cargo_universe programs are only used by Drake's new_release tooling, not
# by any compilation rules. As such, we can put it directly into the WORKSPACE
# instead of into our `//tools/workspace:default.bzl` repositories.
load("@rules_rust//crate_universe:repositories.bzl", "crate_universe_dependencies")  # noqa

crate_universe_dependencies(bootstrap = True)

# XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
# https://github.com/llvm/llvm-project/blob/main/utils/bazel/examples/http_archive/WORKSPACE
# XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Replace with the LLVM commit you want to use.
LLVM_COMMIT = "llvmorg-18.1.1"

# The easiest way to calculate this for a new commit is to set it to empty and
# then run a bazel build and it will report the digest necessary to cache the
# archive and make the build reproducible.
LLVM_SHA256 = "62439f733311869dbbaf704ce2e02141d2a07092d952fc87ef52d1d636a9b1e4"  # noqa

http_archive(
    name = "llvm-raw",
    build_file_content = "# empty",
    patches = [
        "@drake//:aaa-native.patch",
    ],
    sha256 = LLVM_SHA256,
    strip_prefix = "llvm-project-" + LLVM_COMMIT,
    urls = [
        "https://github.com/llvm/llvm-project/archive/{commit}.tar.gz".format(
            commit = LLVM_COMMIT,
        ),
    ],
)

load("@llvm-raw//utils/bazel:configure.bzl", "llvm_configure")

_STUB_REPOS = [
    "terminfo",
    "zlib",
    "zstd",
]

[
    new_local_repository(
        name = "llvm_{name}".format(name = name),
        path = "/tmp/empty",
        build_file_content = """
package(default_visibility = ["//visibility:public"])
cc_library(name = "{name}")
    """.format(name = name),
    )
    for name in _STUB_REPOS
]

llvm_configure(
    name = "llvm-project",
    targets = [
        # We only want clang-format, so we don't need any target architectures.
        # This list should remain present, but empty. If omitted, all targets
        # would be built.
    ],
)
