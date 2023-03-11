#include "drake/bindings/pydrake/math/math_py.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(math, m) {
  // N.B. Docstring contained in `_math_extra.py`.

  py::module::import("pydrake.common");
  py::module::import("pydrake.autodiffutils");
  py::module::import("pydrake.symbolic");

  // The order of these calls matters. Some modules rely on prior definitions.
  internal::DefineMathOverloads(m);
  internal::DefineMathEigenGeometry(m);
  internal::DefineMathRotations(m);
  internal::DefineMathRigidTransform(m);
  internal::DefineMathBaryBspline(m);
  internal::DefineMathEquation(m);
  internal::DefineMathGradient(m);
  internal::DefineMathMatrixUtil(m);

  ExecuteExtraPythonCode(m, true);
}

}  // namespace pydrake
}  // namespace drake
