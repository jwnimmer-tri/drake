#pragma once

/* This file declares the functions that bind the drake::math namespace.
These functions form a complete partition of the drake::math bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per math_py_bary_bspline.cc. */
void DefineMathBaryBspline(py::module m);

/* Defines bindings per math_py_equation.cc. */
void DefineMathEquation(py::module m);

/* Defines bindings per math_py_gradient.cc. */
void DefineMathGradient(py::module m);

/* Defines bindings per math_py_matrix_util.cc. */
void DefineMathMatrixUtil(py::module m);

/* Defines bindings per math_py_overloads.cc. */
void DefineMathOverloads(py::module m);

/* Defines bindings per math_py_rigid_transform.cc. */
void DefineMathRigidTransform(py::module m);

/* Defines bindings per math_py_rotations.cc. */
void DefineMathRotations(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
