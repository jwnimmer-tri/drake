"""
Downloads and unpacks a MOSEK™ archive and makes its headers and
precompiled shared libraries available to be used as a C/C++
dependency.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/mosek:repository.bzl", "mosek_repository")  # noqa
        mosek_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:mosek"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

load(":version.bzl", "SHA256", "VERSION")
load("@drake//tools/workspace:execute.bzl", "execute_or_fail")

def _platform(repository_ctx):
    os_name = repository_ctx.os.name
    os_arch = repository_ctx.os.arch
    NAME_MAP = {
        "linux": "linux",
        "max os x": "osx",
    }
    ARCH_MAP = {
        "aarch64": "aarch64",
        "amd64": "64x86",
        "x86_64": "64x86",
    }
    if os_name not in NAME_MAP or os_arch not in ARCH_MAP:
        fail("Operating system is NOT supported", attr = (os_name, os_arch))
    return NAME_MAP[os_name] + ARCH_MAP[os_arch]

def _impl(repository_ctx):
    # Figure out what platform and version we need to download.
    platform = _platform(repository_ctx)
    major_dot_minor = ".".join(VERSION.split(".")[:2])

    # Download and unpack the MOSEK™ release archive.
    urls = [
        url.format(version = VERSION, platform = platform)
        for url in repository_ctx.attr.mirrors["mosek"]
    ]
    repository_ctx.download_and_extract(
        urls,
        output = repository_ctx.path(""),
        sha256 = SHA256[platform],
        stripPrefix = "mosek/{}".format(major_dot_minor),
    )

    # Something something mac.
    if repository_ctx.os.name == "mac os x":
        # XXX
        mac_files = []
        for file in mac_files:
            file_path = repository_ctx.path("bin/{}".format(file))
            execute_or_fail([
                "install_name_tool",
                "-id",
                file_path,
                file_path,
            ])

    # Add some symlinks.
    repository_ctx.symlink(repository_ctx.attr.build_file, "BUILD.bazel")
    repository_ctx.symlink("tools/platform/{}/bin".format(platform), "bin")
    repository_ctx.symlink("tools/platform/{}/h".format(platform), "h")

mosek_repository = repository_rule(
    implementation = _impl,
    attrs = {
        "mirrors": attr.string_list_dict(
            mandatory = True,
            allow_empty = False,
        ),
        "build_file": attr.label(
            allow_single_file = True,
            default = "@drake//tools/workspace/mosek:package.BUILD.bazel",
        ),
    },
)
