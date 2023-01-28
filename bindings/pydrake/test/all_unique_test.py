import collections
import sys
import types
import unittest

import pydrake.all


class TestAllEachImport(unittest.TestCase):

    @staticmethod
    def _modules_to_test():
        names = []
        for name in sorted(sys.modules.keys()):
            if "._" in name:
                # Private module.
                continue
            if name.endswith(".all"):
                continue
            if name == "pydrake" or name.startswith("pydrake."):
                names.append(name)
        # Fail-fast in case we didn't find all of pydrake. We expect to have
        # many dozens of pydrake modules; we'll check a forgiving lower limit.
        assert len(names) > 10
        return names

    def test_unique_symbol_names(self):
        """Checks all names in pydrake are unique, even across modules."""
        name_to_module = collections.defaultdict(list)
        for module_name in self._modules_to_test():
            print(f"Loading {module_name}")
            module = sys.modules[module_name]
            for symbol_name in dir(module):
                if symbol_name.startswith("_"):
                    continue
                symbol = getattr(module, symbol_name)
                if isinstance(symbol, types.ModuleType):
                    continue
                home = getattr(symbol, "__module__", None)
                if home is not None and sys.modules[home] != module:
                    continue
                name_to_module[symbol_name].append(module_name)
        duplicates = [
            symbol_name
            for symbol_name, modules in name_to_module.items()
            if len(modules) > 1
        ]
        self.assertEqual(len(duplicates), 0, "because\n" + "\n".join([
            f"The name '{name}' is defined by more than one module: "
            f"{name_to_module[name]}"
            for name in sorted(duplicates[:100])
        ]))
