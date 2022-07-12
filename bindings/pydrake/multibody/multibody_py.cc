#include "drake/bindings/pydrake/multibody/multibody_py.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(multibody, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  m.doc() = R"""(
Bindings for Multibody Dynamics.
)""";

  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.common.eigen_geometry");
  py::module::import("pydrake.geometry");
  py::module::import("pydrake.math");
  py::module::import("pydrake.solvers");
  py::module::import("pydrake.symbolic");
  py::module::import("pydrake.systems.framework");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineMultibodyMath(m);
  internal::DefineMultibodyTree(m);
  internal::DefineMultibodyPlant(m);
  internal::DefineMultibodyParsing(m);
  internal::DefineMultibodyInverseKinematics(m);
  internal::DefineMultibodyOptimization(m);
  internal::DefineMultibodyMeshcat(m);
  internal::DefineMultibodyBenchmarks(m);
}

}  // namespace pydrake
}  // namespace drake
