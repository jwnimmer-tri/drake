from pydrake.common import _MangledName
import pydrake.math as _pydrake_math

# These are aliases for backwards compatibility. In new code, please use
# `pydrake.math` directly.
from pydrake.math import (
    AngleAxis,
    AngleAxis_,
    Isometry3,
    Isometry3_,
    Quaternion,
    Quaternion_,
)


def __getattr__(name):
    """Forwards to pydrake.math for backwards compatibility with unpickling."""
    return getattr(_pydrake_math, name)
