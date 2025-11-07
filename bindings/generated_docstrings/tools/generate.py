import argparse
import collections
from pathlib import Path
import tarfile
from xml.etree import ElementTree


class Document:
    """XXX"""

    def __init__(self):
        self._root = Document.Node(name_token=None, parent=None)

    def get_node(
        self,
        *,
        name_tokens: tuple[str],
    ):
        """Returns the node for the given name_chain, creating if necessary."""
        node = self._root
        for name_token in name_tokens:
            node = node.get_child(name_token=name_token)
        return node

    def nodes(self):
        result = []
        worklist = [self._root]
        while worklist:
            item = worklist.pop(0)
            result.append(item)
            worklist.extend(item._children.values())
        return sorted(
            result,
            key=lambda node: node.name_tokens(),
        )

    class Node:
        def __init__(self, *, name_token, parent):
            self._name_token = name_token
            self._parent = parent
            self._children = dict()
            self._docs = list()

        def name_tokens(self):
            result = []
            walker = self
            while walker is not None:
                result.append(walker._name_token)
                walker = walker._parent
            return tuple(reversed(result[:-1]))

        def get_child(
            self,
            *,
            name_token: str,
        ):
            result = self._children.get(name_token)
            if result is None:
                result = Document.Node(name_token=name_token, parent=self)
                self._children[name_token] = result
            return result

        def add_doc(
            self,
            *,
            param_xml: ElementTree.Element | None,
            sphinx: str,
        ):
            self._docs.append((param_xml, sphinx))


class Scraper:
    """XXX"""

    def __init__(self):
        self.doc = Document()

    def _scrape_root(
        self,
        *,
        root: ElementTree.Element,
    ):
        for memberdef in root.iter("memberdef"):
            self._scrape_memberdef(memberdef=memberdef)

    def _scrape_memberdef(
        self,
        *,
        memberdef: ElementTree.Element,
    ):
        kind = memberdef.get("kind")
        if kind in ("define", "friend", "typedef", "variable"):
            return
        assert kind in ("enum", "function"), kind

        prot = memberdef.get("prot")
        if prot == "private":
            return

        if kind == "enum":
            self._scrape_enum(enum=memberdef)
        else:
            assert kind == "function", kind
            self._scrape_function(function=memberdef)

    def _scrape_enum(
        self,
        *,
        enum: ElementTree.Element,
    ):
        pass  # XXX

    def _scrape_function(
        self,
        *,
        function: ElementTree.Element,
    ):
        # Filter out functions that don't need docstrings.
        argsstring = function.find("argsstring").text
        name = function.find("name").text
        qualifiedname = function.find("qualifiedname").text
        if argsstring.endswith("=delete"):
            return
        if name.startswith("~"):
            return
        if not qualifiedname.startswith("drake::"):
            return

        # Special case for constructors.
        is_ctor = qualifiedname.endswith(f"::{name}::{name}")
        if is_ctor:
            self._scrape_ctor(ctor=function)
            return

        # Create the sphinx docstring.
        briefdescription = function.find("briefdescription")
        detaileddescription = function.find("detaileddescription")
        sphinx = "\n\n".join([
            self._transmogrify(description=briefdescription),
            self._transmogrify(description=detaileddescription),
        ])

        # Add it to the documentation tree.
        name_tokens = self._name_tokens(qualifiedname=qualifiedname)
        node = self.doc.get_node(name_tokens=name_tokens)
        node.add_doc(
            param_xml=function.find("param"),
            sphinx=sphinx,
        )

    def _scrape_ctor(
        self,
        *,
        ctor: ElementTree.Element,
    ):
        pass  # XXX

    def _name_tokens(self, *, qualifiedname):
        return tuple(qualifiedname.split("::"))

    def _transmogrify(
        self,
        *,
        description: ElementTree.Element,
    ):
        return ElementTree.tostring(description, encoding="unicode")


def _run(
    *,
    input: Path,
    output: Path,
    hdr_subdir: str,
    exclude_hdr_patterns: list[str],
):
    # Scan the xml input data for paths that match hdr_subdir.
    xml_data = {}
    needle = f'<location file="{hdr_subdir}'
    with tarfile.open(input, mode="r") as tar:
        for name in tar.getnames():
            reader = tar.extractfile(name)
            xml_str = reader.read().decode(encoding="utf-8")
            if needle in xml_str:
                xml_data[name] = xml_str

    # Convert all of that Doxygen documentation to Sphinx.
    scraper = Scraper()
    for xml_str in xml_data.values():
        root = ElementTree.fromstring(xml_str)
        scraper._scrape_root(root=root)

    # Write out the Sphinx docstrings header.
    for node in scraper.doc.nodes():
        for (_, sphinx) in node._docs:
            print(node.name_tokens(), sphinx)

    raise NotImplementedError()


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="XXX",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="XXX",
    )
    parser.add_argument(
        "--hdr_subdir",
        required=True,
        help="XXX",
    )
    parser.add_argument(
        "--exclude_hdr_patterns",
        action="append",
        help="XXX",
    )
    args = parser.parse_args()
    _run(
        input=args.input,
        output=args.output,
        hdr_subdir=args.hdr_subdir,
        exclude_hdr_patterns=args.exclude_hdr_patterns,
    )


assert __name__ == "__main__"
_main()
