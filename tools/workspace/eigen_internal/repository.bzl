# -*- mode: python -*-
# vi: set ft=python :

def _impl(repository_ctx):
    files = (
        ("amalgamator.py", "amalgamator.py"),
        ("amalgamator.bzl", "amalgamator.bzl"),
        ("package.BUILD.bazel", "BUILD.bazel"),
    )
    for old_name, new_name in files:
        old_label = "@drake//tools/workspace/eigen_internal:" + old_name
        repository_ctx.symlink(Label(old_label), new_name)

eigen_internal_repository = repository_rule(
    implementation = _impl,
)
