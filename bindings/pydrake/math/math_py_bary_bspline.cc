#include <cmath>

#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/math/math_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/math/barycentric.h"
#include "drake/math/bspline_basis.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  {
    using Class = BsplineBasis<T>;
    constexpr auto& cls_doc = doc.BsplineBasis;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "BsplineBasis", param, cls_doc.doc);
    cls  // BR
        .def(py::init())
        .def(py::init<int, std::vector<T>>(), py::arg("order"),
            py::arg("knots"), cls_doc.ctor.doc_2args)
        .def(py::init<int, int, KnotVectorType, const T&, const T&>(),
            py::arg("order"), py::arg("num_basis_functions"),
            py::arg("type") = KnotVectorType::kClampedUniform,
            py::arg("initial_parameter_value") = 0.0,
            py::arg("final_parameter_value") = 1.0, cls_doc.ctor.doc_5args)
        .def(py::init<const Class&>(), py::arg("other"))
        .def("order", &Class::order, cls_doc.order.doc)
        .def("degree", &Class::degree, cls_doc.degree.doc)
        .def("num_basis_functions", &Class::num_basis_functions,
            cls_doc.num_basis_functions.doc)
        .def("knots", &Class::knots, cls_doc.knots.doc)
        .def("initial_parameter_value", &Class::initial_parameter_value,
            cls_doc.initial_parameter_value.doc)
        .def("final_parameter_value", &Class::final_parameter_value,
            cls_doc.final_parameter_value.doc)
        .def("FindContainingInterval", &Class::FindContainingInterval,
            py::arg("parameter_value"), cls_doc.FindContainingInterval.doc)
        .def("ComputeActiveBasisFunctionIndices",
            overload_cast_explicit<std::vector<int>, const std::array<T, 2>&>(
                &Class::ComputeActiveBasisFunctionIndices),
            py::arg("parameter_interval"),
            cls_doc.ComputeActiveBasisFunctionIndices
                .doc_1args_parameter_interval)
        .def("ComputeActiveBasisFunctionIndices",
            overload_cast_explicit<std::vector<int>, const T&>(
                &Class::ComputeActiveBasisFunctionIndices),
            py::arg("parameter_value"),
            cls_doc.ComputeActiveBasisFunctionIndices.doc_1args_parameter_value)
        .def(
            "EvaluateCurve",
            [](Class* self, const std::vector<VectorX<T>>& control_points,
                const T& parameter_value) {
              return self->EvaluateCurve(control_points, parameter_value);
            },
            py::arg("control_points"), py::arg("parameter_value"),
            cls_doc.EvaluateCurve.doc)
        .def("EvaluateBasisFunctionI", &Class::EvaluateBasisFunctionI,
            py::arg("i"), py::arg("parameter_value"),
            cls_doc.EvaluateBasisFunctionI.doc)
        .def(py::pickle(
            [](const Class& self) {
              return std::make_pair(self.order(), self.knots());
            },
            [](std::pair<int, std::vector<T>> args) {
              return Class(std::get<0>(args), std::get<1>(args));
            }));
  }
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  // TODO(eric.cousineau): Bind remaining classes for all available scalar
  // types.
  using T = double;
  py::class_<BarycentricMesh<T>>(m, "BarycentricMesh", doc.BarycentricMesh.doc)
      .def(py::init<BarycentricMesh<T>::MeshGrid>(),
          doc.BarycentricMesh.ctor.doc)
      .def("get_input_grid", &BarycentricMesh<T>::get_input_grid,
          doc.BarycentricMesh.get_input_grid.doc)
      .def("get_input_size", &BarycentricMesh<T>::get_input_size,
          doc.BarycentricMesh.get_input_size.doc)
      .def("get_num_mesh_points", &BarycentricMesh<T>::get_num_mesh_points,
          doc.BarycentricMesh.get_num_mesh_points.doc)
      .def("get_num_interpolants", &BarycentricMesh<T>::get_num_interpolants,
          doc.BarycentricMesh.get_num_interpolants.doc)
      .def("get_mesh_point",
          overload_cast_explicit<VectorX<T>, int>(
              &BarycentricMesh<T>::get_mesh_point),
          doc.BarycentricMesh.get_mesh_point.doc_1args)
      .def("get_all_mesh_points", &BarycentricMesh<T>::get_all_mesh_points,
          doc.BarycentricMesh.get_all_mesh_points.doc)
      .def(
          "EvalBarycentricWeights",
          [](const BarycentricMesh<T>* self,
              const Eigen::Ref<const VectorX<T>>& input) {
            const int n = self->get_num_interpolants();
            Eigen::VectorXi indices(n);
            VectorX<T> weights(n);
            self->EvalBarycentricWeights(input, &indices, &weights);
            return std::make_pair(indices, weights);
          },
          doc.BarycentricMesh.EvalBarycentricWeights.doc)
      .def("Eval",
          overload_cast_explicit<VectorX<T>,
              const Eigen::Ref<const MatrixX<T>>&,
              const Eigen::Ref<const VectorX<T>>&>(&BarycentricMesh<T>::Eval),
          doc.BarycentricMesh.Eval.doc_2args)
      .def("MeshValuesFrom", &BarycentricMesh<T>::MeshValuesFrom,
          doc.BarycentricMesh.MeshValuesFrom.doc);

  {
    using Class = KnotVectorType;
    constexpr auto& cls_doc = doc.KnotVectorType;
    py::enum_<Class>(m, "KnotVectorType", py::arithmetic(), cls_doc.doc)
        .value("kUniform", Class::kUniform, cls_doc.kUniform.doc)
        .value("kClampedUniform", Class::kClampedUniform,
            cls_doc.kClampedUniform.doc);
  }
}

}  // namespace

void DefineMathBaryBspline(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
