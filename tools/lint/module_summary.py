import os
from pathlib import Path
import re
import subprocess
from typing import NamedTuple

_INCLUDE_RE = re.compile(r'\s*#\s*include\s*[<"](.*)[>"]')

_SKIP_TREES = [
    "drake/cmake",
    "drake/doc",
    "drake/gen",
    "drake/setup",
    "drake/third_party",
    "drake/tools",
]

_KNOWN_CYCLES = {
    "drake/common": (
        "drake/common/ad",
        "drake/common/ad/internal",
        "drake/common/symbolic",
        "drake/common/symbolic/expression",
    ),
    "drake/multibody/plant": (
        "drake/multibody/fem",
        "drake/multibody/contact_solvers",
        "drake/multibody/contact_solvers/sap",
    ),
    "drake/multibody/tree": (
        "drake/multibody/topology",
    ),
}

class Edge(NamedTuple):
    src: Path
    dst: Path


class SourceTree:
    def __init__(self):
        self.drake = Path(__file__).resolve().parent.parent.parent

    def source_files(self, *, exts, root=None, recursive=True):
        if root is None:
            root = self.drake
        for (dirpath, dirnames, filenames) in os.walk(root):
            # Use a consistent ordering.
            dirnames.sort()
            filenames.sort()
            # The "current" dir is e.g. Path("drake/systems/framework").
            current = Path("drake") / Path(dirpath).relative_to(self.drake)
            # Avoid recursion into unwanted sub-trees.
            if recursive:
                dirnames[:] = [
                    name for name in dirnames
                    if f"{current}/{name}" not in _SKIP_TREES
                       and not name.startswith(".")
                ]
            else:
                dirnames[:] = []
            # Scrape the filenames we want.
            for name in filenames:
                if name.endswith(".cc") or name.endswith(".h"):
                    yield Path(f"{current}/{name}")

    def cc_srcs(self, **kwargs):
        return self.source_files(exts=(".h", ".cc"), **kwargs)

    def cc_dsts(self, src):
        path = self.drake.parent / src
        for line in path.read_text(encoding="utf8").splitlines():
            match = _INCLUDE_RE.match(line)
            if not match:
                continue
            include_filename, = match.groups()
            if not include_filename.startswith("drake/"):
                continue
            yield Path(include_filename)

    def cc_edges(self):
        for src in self.cc_srcs():
            for dst in self.cc_dsts(src):
                yield Edge(src, dst)

    def sloccount(self, path):
        if str(path) == "drake/lcmtypes":
            return 0
        root = self.drake.parent / path
        srcs = list(self.cc_srcs(root=root, recursive=False))
        if not srcs:
            raise RuntimeError(f"No srcs found in {path}")
        result = subprocess.run(
            ["sloccount"] + srcs,
            cwd=self.drake.parent,
            encoding="utf-8",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT)
        if result.returncode != 0:
            print(result.stdout)
            result.check_returncode()
        for line in result.stdout.splitlines():
            if line.startswith("cpp:"):
                return int(line.split()[1])
        raise RuntimeError(f"No cpp found in {path}")

class Graph:
    def __init__(self, *, edges):
        self._reset(edges)
        self.secondaries = {}

    def _reset(self, new_edges):
        self.edges = set([
            edge
            for edge in new_edges
            if edge.src != edge.dst
        ])

    def nodes(self):
        result = set()
        for edge in self.edges:
            result.add(edge.src)
            result.add(edge.dst)
        return result

    def outs(self, node):
        for edge in self.edges:
            if edge.src == node:
                yield edge.dst

    def combine_nodes(self, primary, secondary):
        self.secondaries.setdefault(primary, set()).add(secondary)
        self.respell_nodes(lambda x: primary if x == secondary else x)

    def respell_nodes(self, node_transform):
        self._reset([
            Edge(node_transform(edge.src), node_transform(edge.dst))
            for edge in self.edges
        ])

    def remove_nodes_matching(self, node_pred):
        self.remove_edges_matching(lambda edge: node_pred(edge.src))
        self.remove_edges_matching(lambda edge: node_pred(edge.dst))

    def remove_edges_matching(self, edge_pred):
        self.edges = set([
            edge
            for edge in self.edges
            if not edge_pred(edge)
        ])

    def search(self, start, end, maxstep=2):
        worklist = [(start,)]
        while worklist:
            path = worklist.pop()
            if len(path) > 1 and path[-1] == end:
                yield path
                continue
            for dst in self.outs(path[-1]):
                if dst in path[1:]:
                    continue
                new_path = path + (dst,)
                if len(new_path) < maxstep:
                    worklist.append(new_path)

class Viz:
    def __init__(self, *, tree, graph):
        self.tree = tree
        self.graph = graph

    def token(self, node):
        return str(node).replace(".", "_").replace("/", "__")

    def dot(self):
        lines = []
        lines.append("digraph d {")
        lines.append("ranksep=1;")
        lines.append('TBbalance="max";');
        for node in self.graph.nodes():
            token = self.token(node)
            nodes = [node] + list(self.graph.secondaries.get(node, []))
            label = "\n".join([str(x) for x in nodes])
            sloc = sum([
                self.tree.sloccount(node)
                for node in nodes
            ])
            height = min(max(sloc / 3000, 1), 4)
            lines.append(f'{token} [ '
                         f'label="{label}" '
                         f'shape=box '
                         f'fixedsize=true '
                         f'height={height:.1f} '
                         f'width=4 '
                         f']')
        for (src, dst) in self.graph.edges:
            src_token = self.token(src)
            dst_token = self.token(dst)
            lines.append(f"{src_token} -> {dst_token}")
        lines.append("}")
        return "\n".join(lines) + "\n"

def _canonicalize(path):
    if path.name.endswith(".hpp") and path.name.startswith("lcmt_"):
        return Path("drake/lcmtypes") / path.name
    return path

def main():
    tree = SourceTree()
    graph = Graph(edges=tree.cc_edges())
    for nix in ["benchmarks", "benchmarking", "bindings", "dev", "examples",
                "gen", "profiling", "test", "test_utilities", "tools"]:
        graph.remove_nodes_matching(lambda path: nix in path.parts)
    graph.respell_nodes(_canonicalize)
    graph.respell_nodes(lambda path: path.parent)
    for primary, secondaries in _KNOWN_CYCLES.items():
        for secondary in secondaries:
            graph.combine_nodes(Path(primary), Path(secondary))
    for node in graph.nodes():
        for path in graph.search(node, node, 6):
            print(path)
    dot = Viz(tree=tree, graph=graph).dot()
    Path("out.dot").write_text(dot, encoding="utf-8")

assert __name__ == '__main__'
main()
