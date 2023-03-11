"""
Bindings for ``math``, including overloads for scalar types and basic SE(3)
representations.

Note that arrays of symbolic scalar types, such as ``Variable`` and
``Expression``, are exposed using ``ndarray[object]``, and as such logical
operations are constrained to return boolean values given NumPy's
implementation; this is not desirable, as one should really get a ``Formula``
object. As a workaround, this module provides the following vectorized
operators, following suit with the ``operator`` builtin module:
``lt``, ``le``, ``eq``, ``ne``, ``ge``, and ``gt``.

As an example::

    >>> x = np.array([Variable("x0"), Variable("x1")])
    >>> y = np.array([Variable("y0"), Variable("y1")])
    >>> x >= y
    # This should throw a RuntimeError
    >>> ge(x, y)
    array([<Formula "(x0 >= y0)">, <Formula "(x1 >= y1)">], dtype=object)

"""

# See `ExecuteExtraPythonCode` in `pydrake_pybind.h` for usage details and
# rationale.

import functools
import operator

import numpy as np

from pydrake.autodiffutils import AutoDiffXd as _AutoDiffXd
from pydrake.common import (
    _MangledName,
    pretty_class_name as _pretty_class_name,
)
import pydrake.symbolic as _sym

_sym_cls_list = (
    _sym.Expression,
    _sym.Variable,
)


def _is_elementwise_comparison_error(e):
    return (
        # Newer error message (numpy >= 1.16.0).
        "elementwise comparison failed" in str(e)
        # Older error messages.
        or "elementwise == comparison failed" in str(e)
        or "elementwise != comparison failed" in str(e)
    )


def _best_effort_rich_compare(a, b, *, oper):
    try:
        return oper(a, b)
    except RuntimeError as e:
        if "not call `__bool__` / `__nonzero__` on `Formula`" in str(e):
            if isinstance(a, _sym_cls_list):
                return oper(a, _sym.Expression(b))
            elif isinstance(b, _sym_cls_list):
                return oper(_sym.Expression(a), b)
        raise
    except DeprecationWarning as e:
        # N.B. This is only appears to be triggered for symbolic types.
        if _is_elementwise_comparison_error(e):
            if isinstance(a, np.generic):
                a = float(a)
            elif isinstance(b, np.generic):
                b = float(b)
            else:
                raise RuntimeError("Unexpected condition")
            return oper(a, b)
        raise


def _drake_vectorize(oper, *, doc):
    wrapped = functools.partial(_best_effort_rich_compare, oper=oper)
    return np.vectorize(wrapped, doc=doc)


# As mentioned in top-level, add generic logical operators as ufuncs so that we
# may do comparisons on arrays of any scalar type, without restriction on the
# output type. These are added solely to work around #8315, where arrays of
# Expression can't use direct logical operators (e.g. `<=`) since the output
# type is not bool.
# N.B. Defined in order listed in Python documentation:
# https://docs.python.org/3.6/library/operator.html
lt = _drake_vectorize(operator.lt, doc="Drake's vectorized `lt`")
le = _drake_vectorize(operator.le, doc="Drake's vectorized `le`")
eq = _drake_vectorize(operator.eq, doc="Drake's vectorized `eq`")
ne = _drake_vectorize(operator.ne, doc="Drake's vectorized `ne`")
ge = _drake_vectorize(operator.ge, doc="Drake's vectorized `ge`")
gt = _drake_vectorize(operator.gt, doc="Drake's vectorized `gt`")

# The following values are defined for testing.
_OPERATORS = (lt, le, eq, ne, ge, gt)
# - Equivalent expression when operands are reversed.
_OPERATORS_REVERSE = {
    lt: gt,
    le: ge,
    eq: eq,
    ne: ne,
    ge: le,
    gt: lt,
}


def _retrofit_symbolic_operators():
    """XXX..."""
    operator_names = [
        "abs",
        "acos",
        "arccos",
        "arcsin",
        "arctan",
        "arctan2",
        "asin",
        "atan",
        "atan2",
        "ceil",
        "cos",
        "cosh",
        "exp",
        "floor",
        "inv",
        "isnan",
        "log",
        "max",
        "min",
        "pow",
        "sin",
        "sinh",
        "sqrt",
        "tan",
        "tanh",
        "__abs__",
        "__ceil__",
        "__floor__",
    ]
    for name in operator_names:
        setattr(_sym, name, globals()[name])


_retrofit_symbolic_operators()


def InitializeAutoDiffTuple(*args):
    """Given a series of array_like input arguments, create a tuple of
    corresponding AutoDiff matrices with values equal to the input matrices
    and properly initialized derivative vectors.

    The size of the derivative vector of each element of the matrices in the
    output tuple will be the same, and will equal the sum of the number of
    elements of the matrices in args.

    The 0th element of the derivative vectors will correspond to the
    derivative with respect to the 0th element of the first argument.
    Subsequent derivative vector elements correspond first to subsequent
    elements of the first input argument (traversed first by row, then by
    column), and so on for subsequent arguments.

    This is a pythonic implementation of drake::math::InitializeAutoDiffTuple
    in C++.
    """

    num_derivatives = 0
    for arg in args:
        num_derivatives += np.asarray(arg).size

    autodiff_tuple = []
    deriv_num_start = 0
    for arg in args:
        autodiff_tuple.append(InitializeAutoDiff(arg,
                                                 num_derivatives,
                                                 deriv_num_start))
        deriv_num_start += np.asarray(arg).size

    return tuple(autodiff_tuple)


@np.vectorize
def autodiff_equal_to(a, b, *, semantic=False):
    """
    Provides a structural equality check for arrays of AutoDiffXd scalars, i.e.
    returns True if both the values and derivates are equal.

    Arguments:
        a, b:
            Arrays to compare.
        semantic:
            If False, performs *literal* comparison, meaning the value and
            derivatives must match in both value and shape.
            If True, performs *semantic* comparison, meaning that empty
            derivatives is equivalent to purely zero-valued derivatives.
            Note: Zero-valued derivatives of different size are *not*
            equivalent.
    """
    assert isinstance(a, AutoDiffXd), type(a)
    assert isinstance(b, AutoDiffXd), type(b)
    if a.value() == b.value():
        da = a.derivatives()
        db = b.derivatives()
        if da.shape == db.shape and (da == db).all():
            return True
        da_empty = da.size == 0
        db_empty = db.size == 0
        if semantic and (da_empty or db_empty):
            da_zero = (da == 0.0).all()
            db_zero = (db == 0.0).all()
            if (da_zero and db_empty) or (da_empty and db_zero):
                return True
    return False


def _indented_repr(o):
    """Returns repr(o), with any lines beyond the first one indented +2."""
    return repr(o).replace("\n", "\n  ")


def _remove_float_suffix(typename):
    suffix = "_[float]"
    if typename.endswith(suffix):
        return typename[:-len(suffix)]
    return typename


def _roll_pitch_yaw_repr(rpy):
    cls_name = _remove_float_suffix(_pretty_class_name(type(rpy)))
    return (
        f"{cls_name}("
        f"roll={repr(rpy.roll_angle())}, "
        f"pitch={repr(rpy.pitch_angle())}, "
        f"yaw={repr(rpy.yaw_angle())})")


def _rotation_matrix_repr(R):
    cls_name = _remove_float_suffix(_pretty_class_name(type(R)))
    M = R.matrix().tolist()
    return (
        f"{cls_name}([\n"
        f"  {_indented_repr(M[0])},\n"
        f"  {_indented_repr(M[1])},\n"
        f"  {_indented_repr(M[2])},\n"
        f"])")


def _rigid_transform_repr(X):
    cls_name = _remove_float_suffix(_pretty_class_name(type(X)))
    return (
        f"{cls_name}(\n"
        f"  R={_indented_repr(X.rotation())},\n"
        f"  p={_indented_repr(X.translation().tolist())},\n"
        f")")


def _add_repr_functions():
    for T in [float, _AutoDiffXd, _sym.Expression]:
        RollPitchYaw_[T].__repr__ = _roll_pitch_yaw_repr
        RotationMatrix_[T].__repr__ = _rotation_matrix_repr
        RigidTransform_[T].__repr__ = _rigid_transform_repr


_add_repr_functions()


def __getattr__(name):
    """Rewrites requests for Foo[bar] into their mangled form, for backwards
    compatibility with unpickling.
    """
    return _MangledName.module_getattr(
        module_name=__name__, module_globals=globals(), name=name)
