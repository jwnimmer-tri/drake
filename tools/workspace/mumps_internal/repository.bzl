# -*- mode: python -*-

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)

def _impl(repo_ctx):
    # Find the include path.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        include = "/usr/include"
        libdir = "/usr/lib"
    else:
        fail("Unknown OS")

    # Grab the relevant headers.
    hdrs = [
        "dmumps_c.h",
        "mumps_c_types.h",
        "mumps_compat.h",
        "mumps_int_def.h",
        "mumps_seq/elapse.h",
        "mumps_seq/mpi.h",
        "mumps_seq/mpif.h",
    ]
    for hdr in hdrs:
        repo_ctx.symlink(include + "/" + hdr, "include/" + hdr)

    # Add the BUILD file.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/mumps_internal:package.BUILD.bazel"),
        "BUILD.bazel",
    )

mumps_internal_repository = repository_rule(
    local = True,
    implementation = _impl,
)
