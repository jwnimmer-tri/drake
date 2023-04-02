load(
    "@drake//tools/workspace:pkg_config.bzl",
    "pkg_config_repository",
)

def spdlog_host_repository(
        name,
        licenses = ["notice"],  # MIT
        modname = "spdlog",
        extra_defines = ["HAVE_SPDLOG"],
        extra_deps = ["@fmt"],
        build_epilog = """
load("@drake//tools/install:install.bzl", "install")
install(name = "install")
""",
        **kwargs):
    pkg_config_repository(
        name = name,
        licenses = licenses,
        modname = modname,
        extra_defines = extra_defines,
        extra_deps = extra_deps,
        build_epilog = build_epilog,
        **kwargs
    )
