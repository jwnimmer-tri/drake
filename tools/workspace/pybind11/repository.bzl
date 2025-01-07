load("//tools/workspace:generate_file.bzl", "generate_file")
load("//tools/workspace:github.bzl", "github_archive")

# WIP repository
_REPOSITORY = "jwnimmer-tri/pybind11"

# WIP branch: base-2.13.1-plus-eigen
_COMMIT = "6c0d2092dd69661553bd8f96a22661e471be784c"

_SHA256 = "79fdd576c682d996f7c40205bcb7a4e545da252e73f8e923a47e39bb3d65adeb"

def pybind11_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # WIP local_repository_override = "/home/jwnimmer/jwnimmer-tri/pybind11",
        repository = _REPOSITORY,
        commit = _COMMIT,
        sha256 = _SHA256,
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/check_signature_infection.patch",
        ],
        mirrors = mirrors,
    )

def generate_pybind11_version_py_file(name):
    vars = dict(
        repository = repr(_REPOSITORY),
        commit = repr(_COMMIT),
        sha256 = repr(_SHA256),
    )
    generate_file(
        name = name,
        content = '''# noqa: shebang
"""
Provides information on the external fork of `pybind11` used by `pydrake`.
"""

repository = {repository}
commit = {commit}
sha256 = {sha256}
'''.format(**vars),
    )
