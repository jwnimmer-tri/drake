# -*- mode: python -*-

load("@drake//tools/workspace:github.bzl", "github_download_and_extract")
load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repo_ctx):
    # Find the include path.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        include = "/usr/include/suitesparse"
        lib = "/usr/lib"
    elif os_result.is_macos:
        include = "{}/opt/suite-sparse/include".format(
            os_result.homebrew_prefix,
        )
        lib = "{}/opt/suite-sparse/lib".format(
            os_result.homebrew_prefix,
        )
    elif os_result.is_manylinux:
        github_download_and_extract(
            repo_ctx,
            "DrTimothyAldenDavis/SuiteSparse",
            "v5.1.2",
            repo_ctx.attr.mirrors,
            sha256 = "97dc5fdc7f78ff5018e6a1fcc841e17a9af4e5a35cebd62df6922349bf12959e",  # noqa
        )
        repo_ctx.symlink(
            Label("@drake//tools/workspace/suitesparse:package-manylinux.BUILD.bazel"),  # noqa
            "BUILD.bazel",
        )
        return
    else:
        fail("Unknown OS")

    # Grab the relevant headers.
    hdrs = [
        "SuiteSparse_config.h",
        "amd.h",
    ]
    for hdr in hdrs:
        repo_ctx.symlink(include + "/" + hdr, "include/" + hdr)

    # Declare the libdir.
    repo_ctx.file(
        "vars.bzl",
        content = "LIBDIR = \"{}\"\n".format(lib),
        executable = False,
    )

    # Add the BUILD file.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/suitesparse:package.BUILD.bazel"),
        "BUILD.bazel",
    )

suitesparse_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    local = True,
    implementation = _impl,
)
