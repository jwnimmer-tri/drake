# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_download_and_extract")
load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        build_flavor = "macos"
        repository_ctx.symlink(
            "{}/opt/yaml-cpp/include/yaml-cpp".format(
                os_result.homebrew_prefix,
            ),
            "include/yaml-cpp",
        )
    elif os_result.is_ubuntu:
        build_flavor = "ubuntu-{}".format(os_result.ubuntu_release)
        repository_ctx.symlink("/usr/include/yaml-cpp", "include/yaml-cpp")
    elif os_result.is_manylinux:
        build_flavor = "manylinux"
        github_download_and_extract(
            repository_ctx,
            "jbeder/yaml-cpp",
            "yaml-cpp-0.6.0",
            repository_ctx.attr.mirrors,
            sha256 = "e643119f1d629a77605f02096cc3ac211922d48e3db12249b06a3db810dd8756",  # noqa
        )
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    repository_ctx.symlink(
        Label(
            "@drake//tools/workspace/yaml_cpp:package-{}.BUILD.bazel".format(
                build_flavor,
            ),
        ),
        "BUILD.bazel",
    )

yaml_cpp_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
