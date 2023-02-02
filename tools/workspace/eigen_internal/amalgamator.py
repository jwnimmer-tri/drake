"""XXX
"""

import argparse
import pathlib
import re
import sys


class _Amalgamator:
    """XXX
    """

    def __init__(self, args):
        self.args = args
        self.result = []  # List[str] of amalgamated lines.
        self.visited = set()  # Things we've already included.
        self.quote_include_re = re.compile(r'^\s*#\s*include\s*"([^"]*)"')

    def resolve(self, *, enclosing_directive, directive):
        """XXX
        """
        if enclosing_directive is None:
            rooted_directive = directive
        else:
            start = pathlib.PurePath(enclosing_directive).parent
            path_parts = list(start.joinpath(directive).parts)
            while ".." in path_parts:
                i = path_parts.index("..")
                del path_parts[i-1:i+1]
            rooted_directive = "/".join(path_parts)
        for include in self.args.includes:
            filename = f"{include}/{rooted_directive}"
            if filename in self.args.headers:
                return rooted_directive, filename
        for include in self.args.includes:
            filename = f"{include}/{directive}"
            if filename in self.args.headers:
                return rooted_directive, filename
        raise RuntimeError(
            f"Include of {directive} from {enclosing_directive} resolved to"
            f" {rooted_directive} but no such --header was given as input")

    def include(self, *, enclosing_directive, directive):
        """XXX
        """
        if directive in self.visited:
            return
        if "plugins" not in directive:
            self.visited.add(directive)
        rooted_directive, filename = self.resolve(
            enclosing_directive=enclosing_directive,
            directive=directive)
        with open(filename) as f:
            lines = f.read().splitlines()
        for line in lines:
            match = self.quote_include_re.match(line)
            if not match:
                self.result.append(line)
                continue
            next_directive, = match.groups()
            self.include(
                enclosing_directive=rooted_directive,
                directive=next_directive)

    def run(self):
        """XXX
        """
        for root in self.args.roots:
            self.include(enclosing_directive=None, directive=root)
        for i, line in enumerate(self.result):
            if 'namespace Eigen' in line:
                self.result[i] = line.replace(
                    'Eigen', 'drake_vendor::Eigen')
            elif 'Eigen::' in line:
                self.result[i] = line.replace(
                    'Eigen::', 'drake_vendor::Eigen::')
        return "\n".join(self.result) + "\n"


def _main():
    parser = argparse.ArgumentParser(
        prog="amalgamator", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--output", required=True,
        help="The output filename.")
    parser.add_argument(
        "--root", action="append", dest="roots",
        help="An root directive to start from.")
    parser.add_argument(
        "--header", action="append", dest="headers",
        help="A path to a header file.")
    parser.add_argument(
        "--include", action="append", dest="includes",
        help="An include path to search.")
    args = parser.parse_args()
    output = _Amalgamator(args).run()
    with open(args.output, "w") as f:
        f.write(output)


if __name__ == "__main__":
    _main()
