load("//tools/workspace:pkg_config.bzl", "pkg_config_repository")

def x11_repository(name):
    pkg_config_repository(
        name = name,
        licenses = ["notice"],  # X11/MIT
        modname = "x11",
        defer_error_os_names = ["mac os x"],
        # TODO(jwnimmer-tri) Simplify on 2025-06-01 during deprecation removal.
        extra_deprecation = None if "internal" in name else "DRAKE DEPRECATED: The @x11 external is deprecated and will be removed from Drake on or after 2026-01-01.",  # noqa
    )
