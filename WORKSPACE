# -*- python -*-

# This file marks a workspace root for the Bazel build system. see
# https://bazel.build/ .

workspace(name = "drake")

load("//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_foreign_cc",
    sha256 = "69023642d5781c68911beda769f91fcbc8ca48711db935a75da7f6536b65047f",
    strip_prefix = "rules_foreign_cc-0.6.0",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/0.6.0.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()

# Add some special heuristic logic for using CLion with Drake.
load("//tools/clion:repository.bzl", "drake_clion_environment")

drake_clion_environment()

load("@bazel_skylib//lib:versions.bzl", "versions")

# This needs to be in WORKSPACE or a repository rule for native.bazel_version
# to actually be defined. The minimum_bazel_version value should match the
# version passed to the find_package(Bazel) call in the root CMakeLists.txt.
versions.check(minimum_bazel_version = "4.0")
