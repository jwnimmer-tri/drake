load("@drake//tools/workspace:os.bzl", "os_specific_alias_repository")

def spdlog_repository(
        name,
        # Ignore "mirrors" for backwards compatibility.
        mirrors = None):
    os_specific_alias_repository(
        name = name,
        mapping = {
            "wheel": [
                "spdlog=@spdlog_internal//:spdlog",
                "install=@spdlog_internal//:install",
            ],
            "default": [
                "spdlog=@spdlog_host//:spdlog_host",
                "install=@spdlog_host//:install",
            ],
        },
    )
