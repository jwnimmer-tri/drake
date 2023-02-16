# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")
load(
    "@drake//tools/workspace:github.bzl",
    "setup_github_repository",
)
load(
    "@drake//tools/workspace:pkg_config.bzl",
    "setup_pkg_config_repository",
)

def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    elif True:
        error = setup_github_repository(repo_ctx).error
    elif os_result.is_macos:
        # On macOS, we use fmt from homebrew via pkg-config.
        error = setup_pkg_config_repository(repo_ctx).error
    elif os_result.is_manylinux or os_result.is_macos_wheel:
        # Compile from downloaded github sources.
        error = setup_github_repository(repo_ctx).error
    elif os_result.is_ubuntu and os_result.ubuntu_release == "22.04":
        # On Ubuntu Jammy, we use the host-provided fmt via pkg-config.
        error = setup_pkg_config_repository(repo_ctx).error
    elif os_result.is_ubuntu and os_result.ubuntu_release == "20.04":
        # On Ubuntu Focal, we're using the host-provided spdlog which uses a
        # bundled fmt, so we'll have to reuse that same bundle for ourselves.
        repo_ctx.symlink("/usr/include/spdlog/fmt/bundled", "include/fmt")
        repo_ctx.symlink("include/fmt/LICENSE.rst", "LICENSE.rst")
        repo_ctx.symlink(
            Label("@drake//tools/workspace/fmt:package.BUILD.bazel"),
            "BUILD.bazel",
        )
        error = None
    else:
        fail("Unsupported OS")
    if error != None:
        fail(error)

fmt_repository = repository_rule(
    attrs = {
        # The next two attributes are used only when we take the branch for
        # setup_pkg_config_repository in the above logic.
        "modname": attr.string(
            default = "fmt",
        ),
        "build_epilog": attr.string(
            # When using fmt from pkg-config, there is nothing to install.
            default = """
load("@drake//tools/install:install.bzl", "install")
install(name = "install")
            """,
        ),
        # The remaining attributes are used only when we take the branch for
        # setup_github_repository in the above logic.
        "repository": attr.string(
            default = "fmtlib/fmt",
        ),
        "commit": attr.string(
            # Per https://github.com/gabime/spdlog/releases/tag/v1.5.0 this is
            # the bundled version we should pin, in cases where we're building
            # from source instead of using the host version.
            default = "9.0.0",
        ),
        "commit_pin": attr.int(
            # Per the comment on "commit", above.
            default = 1,
        ),
        "sha256": attr.string(
            default = "9a1e0e9e843a356d65c7604e2c8bf9402b50fe294c355de0095ebd42fb9bd2c5",  # noqa
        ),
        "build_file": attr.label(
            default = "@drake//tools/workspace/fmt:package.BUILD.bazel",
        ),
        "extra_strip_prefix": attr.string(),
        "mirrors": attr.string_list_dict(),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
