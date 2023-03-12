# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import functools
import operator

from pydrake.symbolic import _math_operators


def logical_and(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_and, formulas)


def logical_or(*formulas):
    assert len(formulas) >= 1, "Must supply at least one operand"
    return functools.reduce(__logical_or, formulas)


def _reduce_add(*args):
    return functools.reduce(operator.add, args)


def _reduce_mul(*args):
    return functools.reduce(operator.mul, args)


# These functions are part of `pydrake.math`, but historically we also offered
# an Expression flavor in the `pydrake.symbolic` module.
_MATH_OPERATORS = [
    name
    for name in dir(_math_operators)
    if not name.startswith("_")
]


def _add_math_operator_stubs():
    """Defines math operators in the `pydrake.symbolic` module."""
    for name in _MATH_OPERATORS:
        globals()[name] = getattr(_math_operators, name)


_add_math_operator_stubs()
