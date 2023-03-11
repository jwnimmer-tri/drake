#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/math/math_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/math/continuous_algebraic_riccati_equation.h"
#include "drake/math/continuous_lyapunov_equation.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "drake/math/discrete_lyapunov_equation.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineMathEquation(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  m  // BR
      .def("ContinuousAlgebraicRiccatiEquation",
          py::overload_cast<const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&>(
              &ContinuousAlgebraicRiccatiEquation),
          py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"),
          doc.ContinuousAlgebraicRiccatiEquation.doc_4args_A_B_Q_R)
      .def("RealContinuousLyapunovEquation", &RealContinuousLyapunovEquation,
          py::arg("A"), py::arg("Q"), doc.RealContinuousLyapunovEquation.doc)
      .def("DiscreteAlgebraicRiccatiEquation",
          py::overload_cast<const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&>(
              &DiscreteAlgebraicRiccatiEquation),
          py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"),
          doc.DiscreteAlgebraicRiccatiEquation.doc_4args)
      .def("DiscreteAlgebraicRiccatiEquation",
          py::overload_cast<const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&,
              const Eigen::Ref<const Eigen::MatrixXd>&>(
              &DiscreteAlgebraicRiccatiEquation),
          py::arg("A"), py::arg("B"), py::arg("Q"), py::arg("R"), py::arg("N"),
          doc.DiscreteAlgebraicRiccatiEquation.doc_5args)
      .def("RealDiscreteLyapunovEquation", &RealDiscreteLyapunovEquation,
          py::arg("A"), py::arg("Q"), doc.RealDiscreteLyapunovEquation.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
