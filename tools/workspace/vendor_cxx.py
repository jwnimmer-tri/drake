"""Build system tool that transforms C++ source code for easy vendoring.

Rewrites the include statements, namespace, and symbol visibility with the goal
of producing a completely independent build of some upstream library, even when
statically linking other versions of the library into the same DSO.

Note that this works only on C++ code, not plain C code.
"""

import argparse
import atexit
from enum import Enum
import logging.handlers
import re
import sys

# The C++ text that designates with hidden linker visibility.
_ATTRIBUTE_HIDDEN = '__attribute__ ((visibility ("hidden")))'


# When inserting inline namespaces, we need to decide what to do about each
# line of text from the original file. This enum helps accomplish that.
_Flag = Enum("Flag", [
    "WRAP",
    "NO_WRAP",
    "DONT_CARE",
    "HASH_IF",
    "HASH_ELSE",
    "HASH_ELIF",
    "HASH_ENDIF",
])


class _VendorCxx:
    """Rewrites C++ file contents with specific alterations:

    - Marks all namespaces as hidden.

    - When `add_namespace` is True, wraps an inline namespace "drake_vendor"
    around all of the code in file (but not any #include statements). This
    provides a more thorough vendoring by avoiding ODR conflicts even during
    static linking, but can sometimes be too fussy to enable.

    The call sequence is intended to be init, read, run, write (in that order).
    """

    def __init__(self, *, add_namespace):
        self.add_namespace = add_namespace
        self._input_filename = "<filename>"
        self._lines = []

    def read(self, filename):
        with open(filename, 'r', encoding='utf-8') as f:
            text = f.read()
        self._input_filename = filename
        self._lines = text.split('\n')
        if self._lines[-1] == '':
            self._lines.pop()

    def run(self):
        if self.add_namespace:
            self._do_add_namespace()
        self._do_hide_namespaces()

    def write(self, filename):
        text = '\n'.join(self._lines) + '\n'
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(text)

    def _do_add_namespace():
        self._flags = [None] * len(lines)
        self._parse_initial_flags()
        self._adjust_include_guard_flags()
        self._adjust_all_hash_flags()
        self._adjust_dont_care_flags()
        self._lines = self._with_new_namespaces()

    def _parse_initial_flags(self):
        """Establishes the self._flags as a List[_Flag] with its initial values
        based on the contents of self._lines.
        """
        # Regexs to match various kinds of code patterns.
        is_preprocessor = re.compile(r'^\s*#\s*([a-z]*).*$')
        is_blank = re.compile(r'^\s*$')
        is_blank_cpp_comment = re.compile(r'^\s*//.*$')
        is_blank_c_comment_begin = re.compile(r'^\s*/\*.*$')
        is_c_comment_end = re.compile(r'^.*\*/\s*(.*)$')
        is_extern_c = re.compile(r'^\s*extern\s+"C".*$')
        # Loop over all lines and determine each one's flag.
        assert len(self._flags) == len(self._lines)
        i = 0
        while i < len(self._lines):
            line = self._lines[i]
            # When the prior line has continuation, this line inherits its flag.
            if i > 0 and self._lines[i - 1].endswith('\\'):
                prior_line_flag = self._flags[i - 1]
                if prior_line_flag.name.startswith("HASH"):
                    # But when the prior line was a HASH, it's wrong to flag
                    # this line as *another* HASH token.
                    self._flags[i] = _Flag.DONT_CARE
                else:
                    self._flags[i] = prior_line_flag
                i += 1
                continue
            # Some preprocessor directives are important, others are "don't care".
            preprocessor_match = is_preprocessor.match(line)
            if preprocessor_match:
                command, = preprocessor_match.groups()
                if command in ["include"]:
                    if line.endswith('.inc"'):
                        self._flags[i] = _Flag.DONT_CARE
                    else:
                        self._flags[i] = _Flag.NO_WRAP
                elif command in ["if", "ifdef", "ifndef"]:
                    self._flags[i] = _Flag.HASH_IF
                elif command in ["else"]:
                    self._flags[i] = _Flag.HASH_ELSE
                elif command in ["elif", "elifdef", "elifndef"]:
                    self._flags[i] = _Flag.HASH_ELIF
                elif command in ["endif"]:
                    self._flags[i] = _Flag.HASH_ENDIF
                else:
                    self._flags[i] = _Flag.DONT_CARE
                i += 1
                continue
            # Blank lines (or lines that are blank other than their comments)
            # can go either way.
            if is_blank.match(line) or is_blank_cpp_comment.match(line):
                self._flags[i] = _Flag.DONT_CARE
                i += 1
                continue
            # For C-style comments, consume the entire comment block immediately.
            if is_blank_c_comment_begin.match(line):
                first_c_comment_line = i
                while True:
                    line = self._lines[i]
                    match = is_c_comment_end.match(line)
                    self._flags[i] = _Flag.DONT_CARE
                    i += 1
                    if match:
                        break
                # If the close-comment marker had code after it, we need to go back
                # and set the entire C-style comment to WRAP.
                (trailing,) = match.groups()
                if trailing:
                    for fixup in range(first_c_comment_line, i):
                        self._flags[fixup] = _Flag.WRAP
                continue
            # C ABI must not be namespaced.
            if is_extern_c.match(line):
                if line.endswith(";"):
                    self._flags[i] = _Flag.NO_WRAP
                    i += 1
                    continue
                # Find the line with opening brace.
                while "{" not in self._lines[i]:
                    self._flags[i] = _Flag.NO_WRAP
                    i += 1
                # Find the line with closing brace.
                while "}" not in self._lines[i]:
                    self._flags[i] = _Flag.NO_WRAP
                    i += 1
                    if "{" in self._lines[i]:
                        raise NotImplementedError("extern C ...")
                self._flags[i] = _Flag.NO_WRAP
                i += 1
                continue
            # We MUST wrap all C/C++ code.
            self._flags[i] = _Flag.WRAP
            i += 1
        assert len(self._flags) == len(self._lines)
        assert all([x is not None for x in self._flags])

    @staticmethod
    def _rindex(sequence, x):
        return len(sequence) - 1 - sequence[::-1].index(x)

    def _adjust_include_guard_flags(self):
        """Identifies the canonical C++ include guard pattern (if it exists) in
        self._flags and adjusts the flags for it if so: if everything outside
        the include guard is DONT_CARE then the include guard and everything
        outside it becomes NO_WRAP.
        """
        # Try to find the opener for the canonical pattern.
        try:
            hash_if = self._flags.index(_Flag.HASH_IF)
        except ValueError:
            return
        first_line = self._lines[hash_if]
        second_line = self._lines[hash_if + 1]
        if second_line != first_line.replace("ifndef", "define"):
            return

        # Give up if anything important came before the pattern.
        above = self._flags[:hash_if]
        if any([x != _Flag.DONT_CARE for x in above]):
            return

        # Find the matching endif.
        current_depth = 0
        for i in range(hash_if, len(self._lines)):
            flag = self._flags[i]
            if flag == _Flag.HASH_IF:
                current_depth += 1
            elif flag == _Flag.HASH_ENDIF:
                current_depth -= 1
                if current_depth == 0:
                    break
        assert self._flags[i] == _Flag.HASH_ENDIF
        hash_endif = i

        # Update the guard and its surroundings to be NO_WRAP.
        for i in range(0, hash_if + 1):
            self._flags[i] = _Flag.NO_WRAP
        self._flags[hash_endif] = _Flag.NO_WRAP

    def _adjust_all_hash_flags(self):
        """For preprocessor if-elif-else-endif sections, we need to be careful
        because they cause sections of text to be skipped, which can defeat our
        namespace begin-end pairing.
        """
        # We'll work our way from the most deeply nested if-endif pair(s) down
        # to the least nested. The outermost text in the file will be assigned
        # depth=0. Any #if-#endif pairs found at depth 0 will be assigned
        # depth=1, and the content within the pair will be assigned depth=2.
        # Any #if-#endif pairs found at depth 2 will be assigned depth=3, etc.
        # We'll also assign the EOF marker as depth=0 for convenience.
        depths = [0] * (len(self._flags) + 1)
        current_depth = 0
        for i, flag in enumerate(self._flags):
            if flag == _Flag.HASH_IF:
                current_depth += 1
                depths[i] = current_depth
                current_depth += 1
            elif flag == _Flag.HASH_ENDIF:
                current_depth -= 1
                depths[i] = current_depth
                current_depth -= 1
            else:
                depths[i] = current_depth
        assert all([x >= 0 for x in depths]), "".join([
            f"{depths[i]} {self._lines[i]}\n"
            for i in range(len(self._lines))
        ])
        max_depth = max(depths)
        # The HASH pairs were assigned odd numbers; use the max odd number.
        # We'll adjust all of the e.g. level=5, then level=3, then level=1.
        max_level = max_depth if (max_depth & 1) else max_depth - 1
        for level in range(max_level, 0, -2):
            # Loop over all if-endif pairs at the current level.
            while True:
                try:
                    hash_if_index = depths.index(level)
                except ValueError:
                    # No HASH pairs remain with this level.
                    break
                hash_endif_index = depths.index(level, hash_if_index + 1)
                start = hash_if_index
                end = hash_endif_index + 1
                # Set either WRAP or NO_WRAP on everything [start, end).
                self._adjust_one_hash_if_endif(start, end)
                # Setting this HASH pair's flags reduced this span's depth.
                for i in range(start, end):
                    depths[i] = level - 1
        for i, flag in enumerate(self._flags):
            assert not flag.name.startswith("HASH"), (
                f"Line {i} still has {flag}:\n{self._lines[i]}"
            )

    def _adjust_one_hash_if_endif(self, start, end):
        """Given a [start, end) range for a HASH_IF..HASH_ENDIF span, resets
        the self._flags for that range to definitively WRAP or NO_WRAP. We
        must not have any DONT_CARE near preprocessor branches, because we
        need to keep the begin-end namespaces paired up correctly.
        """
        # Sanity check our input.
        assert self._flags[start] == _Flag.HASH_IF
        assert self._flags[end - 1] == _Flag.HASH_ENDIF
        inside = self._flags[start + 1:end - 1]
        assert _Flag.HASH_IF not in inside, inside
        assert _Flag.HASH_ENDIF not in inside, inside

        # Choose the new flag depending on what's inside.
        # - Leave any existing WRAP or NO_WRAP alone.
        # - All of the HASH_... must have a uniform flag.
        # - Try to repave DONT_CARE to match its neighbords.        
        num_wraps = inside.count(_Flag.WRAP)
        num_no_wraps = inside.count(_Flag.NO_WRAP)
        if num_wraps == 0 and num_no_wraps == 0:
            for i in range(start, end):
                self._flags[i] = _Flag.DONT_CARE
        elif num_wraps == 0:
            assert num_no_wraps > 0
            for i in range(start, end):
                self._flags[i] = _Flag.NO_WRAP
        elif num_no_wraps == 0:
            assert num_wraps > 0
            for i in range(start, end):
                self._flags[i] = _Flag.WRAP
        else:
            assert num_wraps > 0
            assert num_no_wraps > 0
            # Since we have some NO_WRAP already, the guards should be likewise.
            for i in range(start, end):
                if self._flags[i].name.startswith("HASH"):
                    self._flags[i] = _Flag.NO_WRAP
            # Now we need to decide about the DONT_CARE.
            self._adjust_dont_care_flags(start=start, end=end)

    def _adjust_dont_care_flags(self, *, start=None, end=None):
        """XXX
        """
        if start is None:
            start = 0
        if end is None:
            end = len(self._flags)

        # We want to insert inline namespaces such that:
        #
        # - all WRAP lines are enclosed;
        # - no NO_WRAP lines are enclosed;
        # - the only DONT_CARE lines enclosed are surrounded by WRAP.
        #
        # We'll do that by growing the NO_WRAP spans as large as possible.
        # Grow the start-of-file run of NO_WRAP:
        for i in range(start, end):
            if self._flags[i] == _Flag.DONT_CARE:
                self._flags[i] = _Flag.NO_WRAP
            else:
                break

        # Grow the end-of-file run of NO_WRAP:
        for i in range(len(self._flags) - 1, -1, -1):
            if self._flags[i] == _Flag.DONT_CARE:
                self._flags[i] = _Flag.NO_WRAP
            else:
                break

        # Grow any interior regions of NO_WRAP:
        for i in range(start, end):
            if self._flags[i] == _Flag.NO_WRAP:
                # Change all of the immediately prior and subsequent homogeneous
                # runs of DONT_CARE to NO_WRAP.
                for j in range(i - 1, -1, -1):
                    if self._flags[j] == _Flag.DONT_CARE:
                        self._flags[j] = _Flag.NO_WRAP
                    else:
                        break
                for j in range(i + 1, len(self._flags)):
                    if self._flags[j] == _Flag.DONT_CARE:
                        self._flags[j] = _Flag.NO_WRAP
                    else:
                        break

        # Anything remaining is DONT_CARE bookended by WRAP, so we'll WRAP it.
        for i in range(start, end):
            if self._flags[i] == _Flag.DONT_CARE:
                self._flags[i] = _Flag.WRAP

    def _with_new_namespaces(self):
        # We'll add an inline namespace around the C++ code in this file.
        # Designate each line of the file for whether it should be wrapped.
        should_wrap = [x == _Flag.WRAP for x in self._flags]
    
        # Anytime the sense of wrapping switches, we'll insert a line.
        # Do this in reverse order so that the indices into lines[] are stable.
        open_inline = 'inline namespace drake_vendor {'
        close_inline = '}  /* inline namespace drake_vendor */'
        result = list(self._lines)
        for i in range(len(result), -1, -1):
            this_wrap = should_wrap[i] if i < len(result) else False
            prior_wrap = should_wrap[i - 1] if i > 1 else False
            if this_wrap == prior_wrap:
                continue
            insertion = open_inline if this_wrap else close_inline
            result.insert(i, insertion)

        return result

    def _do_hide_namespaces(self):
        """Adds the 'hidden' attribute to all namespaces."""
        # Match either 'namespace foo' or 'namespace foo {'.
        pattern = re.compile(r'^\s*namespace\s+([^{]+?)(\s*{)?$')
        for i, line in enumerate(self._lines):
            match = pattern.match(line)
            if not match:
                continue
            name, brace = match.groups()
            lines[i] = f'namespace {name} {_ATTRIBUTE_HIDDEN}{brace or ""}'


def _configure_logging():
    """Configures logging to buffer in memory until there's an error."""
    stream = logging.StreamHandler()
    stream.setFormatter(logging.Formatter("%(levelname)s: %(message)s"))
    memory = logging.handlers.MemoryHandler(
        target=stream, flushLevel=logging.ERROR, capacity=1e9)
    root = logging.getLogger()
    root.addHandler(memory)
    root.setLevel(logging.INFO)


def _split_pair(arg):
    """Helper function to split ':'-delimited pairs on the command line."""
    old, new = arg.split(':')
    return (old, new)


def _main():
    _configure_logging()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--no-add-namespace', dest='add_namespace', action='store_false',
        help='Amend namespace visibility directly, '
             'without adding a new inline namespace')
    parser.add_argument(
        'rewrite', nargs='+', type=_split_pair,
        help='Filename pairs to rewrite, given as IN:OUT')
    args = parser.parse_args()
    returncode = 0
    for old_filename, new_filename in args.rewrite:
        logging.info(f"Converting {old_filename} to {new_filename} ...")
        try:
            tool = _VendorCxx(add_namespace=args.add_namespace)
            tool.run()
            tool.write(new_filename)
        except Exception as e:
            logging.exception(e)
            returncode = 1
    if returncode == 0:
        # When successful, don't print any log messages.
        logging.getLogger().handlers[-1].buffer[:] = []
    sys.exit(returncode)


if __name__ == '__main__':
    _main()
