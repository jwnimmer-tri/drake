#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/math/math_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/math/cross_product.h"
#include "drake/math/matrix_util.h"
#include "drake/math/quadratic_form.h"
#include "drake/math/wrap_to.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  m.def("wrap_to", &wrap_to<T, T>, py::arg("value"), py::arg("low"),
      py::arg("high"), doc.wrap_to.doc);

  // Cross product
  m.def(
      "VectorToSkewSymmetric",
      [](const Eigen::Ref<const Vector3<T>>& p) {
        return VectorToSkewSymmetric(p);
      },
      py::arg("p"), doc.VectorToSkewSymmetric.doc);
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  // TODO(eric.cousineau): Bind remaining classes for all available scalar
  // types.
  using T = double;

  // Matrix Util.
  m  // BR
      .def(
          "IsSymmetric",
          [](const Eigen::Ref<const MatrixX<T>>& matrix) {
            return IsSymmetric(matrix);
          },
          py::arg("matrix"), doc.IsSymmetric.doc_1args)
      .def(
          "IsSymmetric",
          [](const Eigen::Ref<const MatrixX<T>>& matrix, const T& precision) {
            return IsSymmetric(matrix, precision);
          },
          py::arg("matrix"), py::arg("precision"), doc.IsSymmetric.doc_2args)
      .def(
          "IsPositiveDefinite",
          [](const Eigen::Ref<const Eigen::MatrixXd>& matrix,
              double tolerance) {
            return IsPositiveDefinite(matrix, tolerance);
          },
          py::arg("matrix"), py::arg("tolerance") = 0.0,
          doc.IsPositiveDefinite.doc)
      .def(
          "ToSymmetricMatrixFromLowerTriangularColumns",
          [](const Eigen::Ref<const Eigen::VectorXd>&
                  lower_triangular_columns) {
            return ToSymmetricMatrixFromLowerTriangularColumns(
                lower_triangular_columns);
          },
          py::arg("lower_triangular_columns"),
          doc.ToSymmetricMatrixFromLowerTriangularColumns.doc_dynamic_size);

  // Quadratic Form.
  m  // BR
      .def("DecomposePSDmatrixIntoXtransposeTimesX",
          &DecomposePSDmatrixIntoXtransposeTimesX, py::arg("Y"),
          py::arg("zero_tol"), doc.DecomposePSDmatrixIntoXtransposeTimesX.doc)
      .def("DecomposePositiveQuadraticForm", &DecomposePositiveQuadraticForm,
          py::arg("Q"), py::arg("b"), py::arg("c"), py::arg("tol") = 0,
          doc.DecomposePositiveQuadraticForm.doc)
      .def("BalanceQuadraticForms", &BalanceQuadraticForms, py::arg("S"),
          py::arg("P"), doc.BalanceQuadraticForms.doc);
}

}  // namespace

void DefineMathMatrixUtil(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
