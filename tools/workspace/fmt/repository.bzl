load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def fmt_repository(name, mirrors = None):
    if mirrors != None:
        print("The mirrors=... argument to fmt_repository is deprecated and will be removed on or after 2024-08-01")
    pkg_config_repository(
        name = name,
        modname = "fmt",
        build_epilog = """
# When using fmt from pkg-config, there is nothing to install.
load("@drake//tools/install:install.bzl", "install")
install(name = "install")
""",
    )
