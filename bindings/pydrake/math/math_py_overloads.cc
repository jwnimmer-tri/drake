#include <cmath>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/autodiff_types_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/math/math_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineMathOverloads(py::module m) {
  // Add in query functions that we don't want to bind directly onto
  // the scalar type classes, as mentioned below.
  // TODO(eric.cousineau): Add other query functions.
  // TODO(eric.cousineau): Should these belong on the class for NumPy UFuncs?
  type_visit(
      [&m](auto dummy) {
        using T = decltype(dummy);
        m.def(
            "isnan", [](const T& x) { return isnan(x); }, py::arg("x"));
      },
      CommonScalarPack{});

  // General scalar math overloads.
  // N.B. Additional overloads will be added for AutoDiffXd and Expression
  // via:
  // - `BindAutoDiffMathOverloads` and `BindSymbolicMathOverloads` for
  //   functions that should exist as both overloads in this module *and* as
  //   class members so that NumPy UFuncs for dtype=object can use them, and
  // - `DoScalarDependentDefinitions` defined above, for functions that should
  //   only exist as module-level overloads.
  // TODO(eric.cousineau): If possible, delegate these to explicit NumPy
  // UFuncs, either using __array_ufunc__ or user dtypes.
  // TODO(m-chaturvedi) Add Pybind11 documentation.
  m  // BR
      .def("log", [](double x) { return log(x); })
      .def("abs", [](double x) { return fabs(x); })
      .def("exp", [](double x) { return exp(x); })
      .def("sqrt", [](double x) { return sqrt(x); })
      .def("pow", [](double x, double y) { return pow(x, y); })
      .def("sin", [](double x) { return sin(x); })
      .def("cos", [](double x) { return cos(x); })
      .def("tan", [](double x) { return tan(x); })
      .def("asin", [](double x) { return asin(x); })
      .def("acos", [](double x) { return acos(x); })
      .def("atan", [](double x) { return atan(x); })
      .def(
          "atan2", [](double y, double x) { return atan2(y, x); }, py::arg("y"),
          py::arg("x"))
      .def("sinh", [](double x) { return sinh(x); })
      .def("cosh", [](double x) { return cosh(x); })
      .def("tanh", [](double x) { return tanh(x); })
      .def("min", [](double x, double y) { return fmin(x, y); })
      .def("max", [](double x, double y) { return fmax(x, y); })
      .def("ceil", [](double x) { return ceil(x); })
      .def("floor", [](double x) { return floor(x); });

  // General vectorized / matrix overloads.
  m  // BR
      .def("inv", [](const Eigen::MatrixXd& X) -> Eigen::MatrixXd {
        return X.inverse();
      });

  // See TODO in corresponding header file - these should be removed soon!
  pydrake::internal::BindAutoDiffMathOverloads(&m);
  pydrake::internal::BindSymbolicMathOverloads<symbolic::Expression>(&m);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
