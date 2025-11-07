"""...XXX..."""

import argparse
import io
import os
from pathlib import Path
import tarfile
import tempfile

from doc.doxygen_cxx.build import _build


def strip_tar_metadata(info: tarfile.TarInfo):
    """Removes some metadata (owner, timestamp) from a TarInfo."""
    info.uid = info.gid = 0
    info.uname = info.gname = "root"
    info.mtime = 0
    info.pax_headers = {}
    return info


def _run(*, temp_dir: Path, out_dir: Path, output: Path):
    # Run Doxygen.
    _build(
        out_dir=out_dir,
        temp_dir=temp_dir,
        modules=[],
        quick=False,
        format="xml",
    )
    xml_dir = out_dir / "xml"

    # Collect the list of useful xml output files.
    useful_kinds = ["class", "group", "namespace", "struct"]
    useful_files = []
    for root, dirs, files in os.walk(xml_dir):
        assert len(dirs) == 0
        for name in sorted(files):
            if any([name.startswith(x) for x in useful_kinds]):
                useful_files.append(name)

    # Save them into a single tar file for easier handling downstream.
    tar_buffer = io.BytesIO()
    tar_writer = tarfile.open(mode="w", fileobj=tar_buffer)
    for name in useful_files:
        tar_writer.add(
            xml_dir / name, name, recursive=False, filter=strip_tar_metadata
        )
    tar_writer.close()
    tar_buffer.seek(0)
    output.write_bytes(tar_buffer.read())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        metavar="TARFILE",
        required=True,
    )
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(
        dir=os.environ.get("TEST_TMPDIR"), prefix="doc_builder_temp_xml_"
    ) as temp_base:
        temp_dir = Path(temp_base) / "temp"
        temp_dir.mkdir()
        out_dir = Path(temp_base) / "out"
        out_dir.mkdir()
        _run(
            temp_dir=temp_dir,
            out_dir=out_dir,
            output=Path(args.output),
        )


assert __name__ == "__main__"
main()
