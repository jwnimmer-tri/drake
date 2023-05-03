# -*- python -*-

def _impl(repository_ctx):
    repository_ctx.download_and_extract(
        url = [
            x.format(archive = repository_ctx.attr.archive)
            for x in repository_ctx.attr.mirrors.get("mumps")
        ],
        sha256 = repository_ctx.attr.sha256,
        stripPrefix = repository_ctx.attr.strip_prefix,
    )

    repository_ctx.symlink(
        Label("@drake//tools/workspace/mumps_internal:package.BUILD.bazel"),
        "BUILD.bazel",
    )

mumps_internal_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
        "archive": attr.string(default = "MUMPS_5.5.1.tar.gz"),
        "sha256": attr.string(default = "1abff294fa47ee4cfd50dfd5c595942b72ebfcedce08142a75a99ab35014fa15"),  # noqa
        "strip_prefix": attr.string(default = "MUMPS_5.5.1"),
    },
    implementation = _impl,
)
