def _impl(repository_ctx):
    repository_ctx.symlink(
        # TODO(jwnimmer-tri) Simplify on 2025-06-01 during deprecation removal.
        Label("@drake//tools/workspace/opencl:package.BUILD.bazel") if "internal" in repository_ctx.name else Label("@drake//tools/workspace/opencl:package-deprecated.BUILD.bazel"),  # noqa
        "BUILD.bazel",
    )

opencl_repository = repository_rule(
    implementation = _impl,
)
