load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def musl_internal_repository(name):
    http_archive(
        name = name,
        url = "https://musl.libc.org/releases/musl-1.2.5.tar.gz",
        strip_prefix = "musl-1.2.5",
        sha256 = "a9a118bbe84d8764da0ea0d28b3ab3fae8477fc7e4085d90102b8596fc7c75e4",  # noqa
        build_file = "@drake//tools/workspace/musl_internal:package.BUILD.bazel",  # noqa
    )
