def _impl(repo_ctx):
    build_files = {}
    for target in repo_ctx.attr.available_targets:
        subdir, name = target[2:].split(":")
        path = "{}/BUILD.bazel".format(subdir)
        if path in build_files:
            text = build_files[path]
        else:
            text = "package(default_visibility = [\"//visibility:public\"])\n"
            text += "load(\"@drake//tools/workspace/abseil_cpp_internal:defs.bzl\", \"absl_cc_library\")\n"
        text += "absl_cc_library({})\n".format(", ".join([
            "name = {}".format(repr(name)),
            "deps = [{}]".format(repr("@module_abseil//{}:{}".format(subdir, name))),
        ]))
        build_files[path] = text
    for path, text in build_files.items():
        repo_ctx.file(path, text)

abseil_cpp_internal_repository = repository_rule(
    attrs = {
        "available_targets": attr.string_list(
            default = [
                "//absl/container:flat_hash_set",
                "//absl/container:inlined_vector",
            ],
        ),
    },
    implementation = _impl,
)


# load("//tools/workspace:github.bzl", "github_archive")

# def abseil_cpp_internal_repository(
#         name,
#         mirrors = None):
#     github_archive(
#         name = name,
#         repository = "abseil/abseil-cpp",
#         upgrade_type = "release",
#         commit = "20260526.0",
#         sha256 = "6e1aee535473414164bf83e4ebc40240dec71a4701f8a642d906e95bea1aea0c",  # noqa
#         patches = [
#             ":patches/upstream/include_bmi2intrin.patch",
#             ":patches/hidden_visibility.patch",
#             ":patches/inline_namespace.patch",
#         ],
#         patch_cmds = [
#             "echo 'exports_files([\"drake_repository_metadata.json\"])' >> BUILD.bazel",  # noqa
#             # Force linkstatic = 1 everywhere. First, remove the few existing
#             # uses so that we don't get "duplicate kwarg" errors. Then, add it
#             # anywhere that linkopts already appears.
#             "sed -i -e 's|linkstatic = 1,||; s|linkopts = |linkstatic = 1, linkopts = |' $(find absl -name BUILD.bazel)",  # noqa
#         ],
#         mirrors = mirrors,
#     )
