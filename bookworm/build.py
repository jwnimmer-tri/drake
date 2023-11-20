#!/usr/bin/env python3

"""This is small wrapper to run the Dockerfile against the current checkout of
Drake. An actual Debian Bookworm debian/rules would not need this.
"""

import os.path
import subprocess
import tarfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)


def _strip_tar_metadata(info):
    """
    Removes some metadata (owner, timestamp) from a TarInfo.
    """
    info.uid = info.gid = 0
    info.uname = info.gname = 'root'
    info.mtime = 0
    info.pax_headers = {}
    return info


def _add_to_tar(tar, name, parent_path):
    """
    Adds files or directories to the specified tar file.
    """
    tar_path = os.path.join(parent_path, name)
    full_path = os.path.join(ROOT, parent_path, name)

    if os.path.isdir(full_path):
        for f in sorted(os.listdir(full_path)):
            _add_to_tar(tar, f, os.path.join(parent_path, name))
    else:
        tar.add(full_path, tar_path, recursive=False,
                filter=_strip_tar_metadata)


def _create_source_tar():
    """
    Creates a tarball of the repository working tree.
    """
    out = tarfile.open('bookworm/image/drake-src.tar', 'w')

    # Walk the git root and archive almost every file we find.
    for f in sorted(os.listdir('.')):
        # Exclude build and VCS directories.
        if f == '.git' or f == 'user.bazelrc' or f.startswith('bazel-'):
            continue

        # Exclude host-generated setup files.
        if f == 'gen':
            continue

        # Exclude ourself.
        if f == 'bookworm':
            continue

        _add_to_tar(out, f, '')

    out.close()


def _docker(*args, stdout=None):
    """
    Runs a docker command.
    The value of `stdout` is passed through to the subprocess module.
    Blocks until completion and returns a CompletedProcess instance.
    """
    command = ['docker'] + list(args)
    environment = os.environ.copy()
    environment['DOCKER_BUILDKIT'] = '1'
    return subprocess.run(command, check=True, stdout=stdout,
                          cwd='bookworm', env=environment)


def _main():
    _create_source_tar()
    _docker('build', '.')


assert __name__ == '__main__'
_main()
