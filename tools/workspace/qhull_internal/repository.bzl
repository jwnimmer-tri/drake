load("//tools/workspace:github.bzl", "github_archive")

def qhull_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "qhull/qhull",
        commit = "2020.2",
        sha256 = "59356b229b768e6e2b09a701448bfa222c37b797a84f87f864f97462d8dbc7c5",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            # The Qhull license requires us to publish the author, date, and
            # reason for changes. Ensure that any new patch files added here
            # contain that information in their opening commentary.
            ":patches/upstream/cxx20.patch",
            ":patches/disable_dead_code.patch",
            ":patches/disable_static_global_log.patch",
            ":patches/vendor.patch",
        ],
        patch_cmds = [
            # XXX document me
            # XXX ship the patch in our release FML stupid fucking license
            "sed -i -e 's|<iostream>|<drake_std_cout_cerr.h>|;' $(find src/libqhullcpp -name '*.[ch]*')",  # noqa
        ],
        mirrors = mirrors,
    )
