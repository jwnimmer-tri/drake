"""Common library to provide a reusable main() routine for all of our
documentation generation tools.
"""

import argparse
import filecmp
import functools
from http.server import SimpleHTTPRequestHandler
import os.path
from os.path import join
from socketserver import ThreadingTCPServer
import shlex
import shutil
import subprocess
from subprocess import PIPE, STDOUT
import tempfile
import threading
import time

from bazel_tools.tools.python.runfiles import runfiles

# This global variable can be toggled by our main() function.
_verbose = False


def verbose():
    """Returns True iff doc builds should produce detailed console output."""
    return _verbose


def symlink_input(filegroup_resource_path, temp_dir, strip_prefix=None,
                  copy=False):
    """Symlinks a rule's input data into a temporary directory.

    This is useful both to create a hermetic set of inputs to pass to a
    documentation builder, or also in case we need to adjust the input data
    before passing it along.

    Args:
        filegroup_resource_path: Names a file created by enumerate_filegroup
          (in defs.bzl) which contains resource paths.
        temp_dir: Destination directory, which must already exist.
        strip_prefix: Optional; a list[str] of candidate strings to remove
          from the resource path when linking into temp_dir.  The first match
          wins, and it is valid for no prefixes to match.
        copy: Optional; if True, copies rather than linking.
    """
    assert os.path.isdir(temp_dir)
    manifest = runfiles.Create()
    with open(manifest.Rlocation(filegroup_resource_path)) as f:
        input_filenames = f.read().splitlines()
    for name in input_filenames:
        orig_name = manifest.Rlocation(name)
        assert os.path.exists(orig_name), name
        dest_name = name
        for prefix in (strip_prefix or []):
            if dest_name.startswith(prefix):
                dest_name = dest_name[len(prefix):]
                break
        temp_name = join(temp_dir, dest_name)
        os.makedirs(os.path.dirname(temp_name), exist_ok=True)
        if copy:
            shutil.copy(orig_name, temp_name)
        else:
            os.symlink(orig_name, temp_name)


def check_call(args, *, cwd=None):
    """Runs a subprocess command, raising an exception iff the process fails.

    Obeys the command-line verbosity flag for console output:
    - when in non-verbose mode, shows output only in case of an error;
    - when in verbose mode, shows the command-line and live output.

    Args:
        args: Passed to subprocess.run(args=...).
    """
    env = dict(os.environ)
    env["LC_ALL"] = "en_US.UTF-8"
    echo = "+ " + " ".join([shlex.quote(x) for x in args])
    if verbose():
        print(echo, flush=True)
        proc = subprocess.run(args, cwd=cwd, env=env, stderr=STDOUT)
    else:
        proc = subprocess.run(args, cwd=cwd, env=env, stderr=STDOUT,
                              stdout=PIPE, encoding='utf-8')
        if proc.returncode != 0:
            print(echo, flush=True)
            print(proc.stdout, end='', flush=True)
    proc.check_returncode()


def perl_cleanup_html_output(*, out_dir, extra_perl_statements=None):
    """Runs a cleanup pass over all HTML output files, using a set of built-in
    fixups. Calling code may pass its own extra statements, as well.
    """
    # Collect the list of all HTML output files.
    html_files = []
    for dirpath, _, filenames in os.walk(out_dir):
        for filename in filenames:
            if filename.endswith(".html"):
                html_files.append(os.path.relpath(
                    join(dirpath, filename), out_dir))

    # Figure out what to do.
    default_perl_statements = [
        # Add trademark hyperlink.
        r's#™#<a href="/tm.html">™</a>#g;',
    ]
    perl_statements = default_perl_statements + (extra_perl_statements or [])
    for x in perl_statements:
        assert x.endswith(';'), x

    # Do it.
    while html_files:
        # Work in batches of 100, so we don't overflow the argv limit.
        first, html_files = html_files[:100], html_files[100:]
        check_call(["perl", "-pi", "-e", "".join(perl_statements)] + first,
                   cwd=out_dir)


def _sync(*, src, dst):
    """Syncs (copies) the contents of src into dst, also removing any files
    from dst that were not present in src. Returns a list of added or modified
    files (as relative paths) in case dst was non-empty to begin with.

    We use the styleguide-violating abbreviations `src` and `dst` to align
    with the Python shutil.copytree convention.
    """
    # Scan the src and dst directories.
    with os.scandir(src) as it:
        src_entries = list(it)
    os.makedirs(dst, exist_ok=True)
    with os.scandir(dst) as it:
        initial_dst_entries = list(it)

    # Loop over src, copying the contents.
    result = []
    for src_entry in src_entries:
        src_name = join(src, src_entry.name)
        dst_name = join(dst, src_entry.name)
        if src_entry.is_dir():
            sub_result = _sync(src=src_name, dst=dst_name)
            result.extend([
                join(src_entry.name, sub_name)
                for sub_name in sub_result
            ])
        else:
            noop = (os.path.exists(dst_name)
                    and filecmp.cmp(src_name, dst_name, shallow=False))
            if not noop:
                shutil.copy2(src=src_entry, dst=dst_name)
                result.append(src_entry.name)

    # Loop over initial_dst, removing anything no longer in src.
    src_names = set([x.name for x in src_entries])
    for dst_entry in initial_dst_entries:
        if dst_entry.name in src_names:
            continue
        print(f"rm {dst_entry.name}")

    # Don't report "changed files" when the dst started out empty.
    if len(initial_dst_entries) == 0:
        result = []

    return result


def _call_build(*, build, out_dir, inject_images):
    """Calls build() into out_dir, while also supplying a temp_dir.

    The files are built into a scratch folder (the build might be slow) and
    then moved into out_dir (which should be quick) support of incremental
    refreshes when in preview mode.

    When inject_images is True, the Jekyll sites images are injected into the
    result, which is necessary because C++ and Python API guides re-use those
    images.

    The result of `build()` (which is the list of suggested preview URLs) is
    returned as the result of this function.
    """
    with tempfile.TemporaryDirectory(
            dir=os.environ.get("TEST_TMPDIR"),
            prefix="doc_builder_temp_") as temp_dir:
        with tempfile.TemporaryDirectory(
                dir=os.environ.get("TEST_TMPDIR"),
                prefix="doc_builder_scratch_") as scratch_dir:
            # Call the specific builder (jekyll, doxygen, sphix, etc.).
            result = build(out_dir=scratch_dir, temp_dir=temp_dir)

            # Conditionally add some extra images.
            if inject_images:
                symlink_input(
                    "drake/doc/header_and_footer_images.txt",
                    strip_prefix=["drake/doc/"],
                    temp_dir=scratch_dir)

            # Sync the files from scratch to output, telling the user in case
            # the preview site has changed.
            changes = _sync(src=scratch_dir, dst=out_dir)
            if changes:
                print(f"Re-generated {len(changes)} files.", flush=True)
    return result


class _PreviewBuilder:
    """Helper class that knows how to (re)build a site for local preview.

    Optionally can start() this object to launch a background thread that
    will rebuild the site repeatedly so local preview can refresh without
    the user constantly re-running the preview command line.
    """

    def __init__(self, *, build, out_dir, inject_images):
        self._build = build
        self._out_dir = out_dir
        self._inject_images = inject_images
        self._thread = None
        self._should_stop = False

    def start(self):
        self.thread = threading.Thread(target=self._loop)
        self.thread.start()

    def stop(self):
        self._should_stop = True
        if self._thread:
            self._thread.join()
            self._thread = None

    def _loop(self):
        while not self._should_stop:
            self.build()

    def build(self):
        return _call_build(
            build=self._build,
            out_dir=self._out_dir,
            inject_images=self._inject_images)


class _HttpHandler(SimpleHTTPRequestHandler):
    """An HTTP handler without logging."""

    def log_message(*_):
        pass

    def log_request(*_):
        pass


def _do_preview(*, build, subdir, port, watch):
    """Implements the "serve" (http) mode of main().

    Args:
        build: Same as per main().
        subdir: Same as per main().
        port: Local port number to serve on, per the command line.
    """
    print("Generating documentation preview ...")
    with tempfile.TemporaryDirectory(
            prefix="doc_builder_preview_") as preview_dir:
        if subdir:
            out_dir = join(preview_dir, subdir)
            os.mkdir(out_dir)
        else:
            out_dir = preview_dir
        preview_builder = _PreviewBuilder(
            build=build,
            out_dir=out_dir,
            inject_images=bool(subdir))
        pages = preview_builder.build()
        assert len(pages) > 0
        if watch:
            preview_builder.start()
        os.chdir(preview_dir)
        print(f"The files have temporarily been generated into {preview_dir}")
        print()
        print("Serving at the following URLs for local preview:")
        print()
        for page in pages:
            print(f"  http://127.0.0.1:{port}/{join(subdir, page)}")
        print()
        print("Use Ctrl-C to exit.")
        ThreadingTCPServer.allow_reuse_address = True
        server = ThreadingTCPServer(("127.0.0.1", port), _HttpHandler)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print()
            preview_builder.stop()
            return


def _do_generate(*, build, out_dir, on_error):
    """Implements the "generate" (file output) mode of main().
    Args:
        build: Same as per main().
        out_dir: Directory to generate into, per the command line.
        on_error: Callback function to report problems with out_dir.
    """
    if out_dir == "<test>":
        out_dir = join(os.environ["TEST_TMPDIR"], "_builder_out")
    if not os.path.isabs(out_dir):
        on_error(f"--out_dir={out_dir} is not an absolute path")
    if os.path.exists(out_dir):
        if len(os.listdir(out_dir)) > 0:
            on_error(f"--out_dir={out_dir} is not empty")
    else:
        if verbose():
            print(f"+ mkdir -p {out_dir}", flush=True)
        os.makedirs(out_dir)
    print("Generating HTML ...")
    pages = _call_build(build=build, out_dir=out_dir, inject_images=False)
    assert len(pages) > 0
    # Disallow symlinks in the output dir.
    for root, dirs, _ in os.walk(out_dir):
        for one_dir in dirs:
            for entry in os.scandir(f"{root}/{one_dir}"):
                assert not entry.is_symlink(), entry.path
    print("... done")


def main(*, build, subdir, description, supports_modules=False,
         supports_quick=False):
    """Reusable main() function for documentation binaries; processes
    command-line arguments and generates documentation.

    Args:
      build: Callback function to compile the documentation.
      subdir: A subdirectory to use when offering preview mode on a local web
        server; this does NOT affect the --out_dir path.
      description: Main help str for argparse; typically the caller's __doc__.
      supports_modules: Whether build() has a modules=list[str] argument.
      supports_quick: Whether build() has a quick=bool argument.
    """
    parser = argparse.ArgumentParser(description=description)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--serve", action="store_true",
        help="Serve the documentation on the given PORT for easy preview.")
    group.add_argument(
        "--out_dir", type=str, metavar="DIR",
        help="Generate the documentation to the given output directory."
        " The DIR must be an absolute path."
        " If DIR already exists, then it must be empty."
        " (For regression testing, the DIR can be the magic value <test>,"
        " in which case a $TEST_TMPDIR subdir will be used.)")
    parser.add_argument(
        "--watch", action="store_true",
        help="Rebuild the preview as the input files change, so that a"
        " browser reload will reflect new source-file changes.")
    parser.add_argument(
        "--port", type=int, metavar="PORT", default=8000,
        help="Use a non-default PORT when serving for preview.")
    parser.add_argument(
        "--verbose", action="store_true",
        help="Echo detailed commands, progress, etc. to the console")
    if supports_modules:
        parser.add_argument(
            "module", nargs="*",
            help="Limit the generated documentation to only these modules and "
            "their children.  When none are provided, all will be generated. "
            "For example, specify drake.math or drake/math for the C++ "
            "module, or pydrake.math or pydrake/math for the Python module.")
    if supports_quick:
        parser.add_argument(
            "--quick", action="store_true", default=False,
            help="Omit from the output items that are slow to generate. "
            "This yields a faster preview, but the output will be incomplete.")
    args = parser.parse_args()
    if args.verbose:
        global _verbose
        _verbose = True
    curried_build = build
    if supports_modules:
        canonicalized_modules = [
            x.replace('/', '.')
            for x in args.module
        ]
        curried_build = functools.partial(
            curried_build, modules=canonicalized_modules)
    if supports_quick:
        curried_build = functools.partial(
            curried_build, quick=args.quick)
    if args.out_dir is None:
        assert args.serve
        _do_preview(build=curried_build, subdir=subdir, port=args.port,
                    watch=args.watch)
    else:
        assert not args.watch
        _do_generate(build=curried_build, out_dir=args.out_dir,
                     on_error=parser.error)


if __name__ == '__main__':
    main()
