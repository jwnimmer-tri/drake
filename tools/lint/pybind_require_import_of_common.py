import argparse
import sys
import textwrap

_IMPORT_STATEMENT = 'py::module::import("pydrake.common");'


def _imports_common(filenames):
    for filename in filenames:
        with open(filename, "r") as f:
            text = f.read()
        if _IMPORT_STATEMENT in text:
            return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filenames", type=str, nargs="+")
    args = parser.parse_args()
    assert len(args.filenames) > 0

    filenames = sorted(args.filenames)
    if _imports_common(args.filenames):
        sys.exit(0)
    else:
        print("The following source files do not directly import ")
        print("`pydrake.common`:")
        print(textwrap.indent("\n".join(filenames), "  "))
        print()
        print("If this is user-facing code, please add the following code to ")
        print("the top of the main C++ binding function:")
        print()
        print(f"  {_IMPORT_STATEMENT}")
        print()
        print("If this is not user-facing code, please set ")
        print("drake_pybind_library(..., require_import_of_common = False) ")
        print("for your module.")
        print()
        sys.exit(1)


if __name__ == "__main__":
    main()
