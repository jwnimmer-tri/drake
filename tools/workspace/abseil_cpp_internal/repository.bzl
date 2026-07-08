AVAILABLE_TARGETS = [
    "//absl/container:flat_hash_map",
    "//absl/container:flat_hash_set",
    "//absl/container:inlined_vector",
    "//absl/strings:strings",
]

def _impl(repo_ctx):
    build_files = {}
    for target in AVAILABLE_TARGETS:
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
    implementation = _impl,
)
