load(
    "//tools/workspace/crate_universe/lock/details:crates.bzl",
    "crate_repositories",
)

def _impl(module_ctx):
    direct_deps = crate_repositories()
    return module_ctx.extension_metadata(
        root_module_direct_deps = [repo.repo for repo in direct_deps],
        root_module_direct_dev_deps = [],
    )

crate_universe = module_extension(
    doc = "Internal use only.",
    implementation = _impl,
)
