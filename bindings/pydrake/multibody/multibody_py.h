#pragma once

/* This file declares the functions that bind the drake::multibody namespace.
These functions form a complete partition of the drake::multibody bindings.

The implementations of these functions are parceled out into various *.cc
files as indicated in each function's documentation. */

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

/* Defines bindings per multibody_py_benchmarks.cc. */
void DefineMultibodyBenchmarks(py::module m);

/* Defines bindings per multibody_py_inverse_kinematics.cc. */
void DefineMultibodyInverseKinematics(py::module m);

/* Defines bindings per multibody_py_math.cc. */
void DefineMultibodyMath(py::module m);

/* Defines bindings per multibody_py_meshcat.cc. */
void DefineMultibodyMeshcat(py::module m);

/* Defines bindings per multibody_py_optimization.cc. */
void DefineMultibodyOptimization(py::module m);

/* Defines bindings per multibody_py_parsing.cc. */
void DefineMultibodyParsing(py::module m);

/* Defines bindings per multibody_py_plant.cc. */
void DefineMultibodyPlant(py::module m);

/* Defines bindings per multibody_py_tree.cc. */
void DefineMultibodyTree(py::module m);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
