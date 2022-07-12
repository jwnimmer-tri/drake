"""Shim module that provides vestigial names for pydrake.multibody.

Prefer not to use this import path in new code; all of the code in
this module can be imported from pydrake.solvers directly.

This module will be deprecated at some point in the future.
"""

from pydrake.multibody import (
    SpatialAcceleration,
    SpatialAcceleration_,
    SpatialForce,
    SpatialForce_,
    SpatialMomentum,
    SpatialMomentum_,
    SpatialVelocity,
    SpatialVelocity_,
)
