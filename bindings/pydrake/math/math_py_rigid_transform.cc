#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

template <typename T>
void DoBind(py::module m, T) {
  py::tuple param = GetPyParam<T>();

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  {
    using Class = RigidTransform<T>;
    constexpr auto& cls_doc = doc.RigidTransform;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "RigidTransform", param, cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<const RotationMatrix<T>&, const Vector3<T>&>(),
            py::arg("R"), py::arg("p"), cls_doc.ctor.doc_2args_R_p)
        .def(py::init<const RollPitchYaw<T>&, const Vector3<T>&>(),
            py::arg("rpy"), py::arg("p"), cls_doc.ctor.doc_2args_rpy_p)
        .def(py::init<const Eigen::Quaternion<T>&, const Vector3<T>&>(),
            py::arg("quaternion"), py::arg("p"),
            cls_doc.ctor.doc_2args_quaternion_p)
        .def(py::init<const Eigen::AngleAxis<T>&, const Vector3<T>&>(),
            py::arg("theta_lambda"), py::arg("p"),
            cls_doc.ctor.doc_2args_theta_lambda_p)
        .def(py::init<const RotationMatrix<T>&>(), py::arg("R"),
            cls_doc.ctor.doc_1args_R)
        .def(py::init<const Vector3<T>&>(), py::arg("p"),
            cls_doc.ctor.doc_1args_p)
        .def(py::init<const Isometry3<T>&>(), py::arg("pose"),
            cls_doc.ctor.doc_1args_pose)
        .def(py::init<const MatrixX<T>&>(), py::arg("pose"),
            cls_doc.ctor.doc_1args_constEigenMatrixBase)
        .def("set", &Class::set, py::arg("R"), py::arg("p"), cls_doc.set.doc)
        .def("SetFromIsometry3", &Class::SetFromIsometry3, py::arg("pose"),
            cls_doc.SetFromIsometry3.doc)
        .def_static("Identity", &Class::Identity, cls_doc.Identity.doc)
        .def("rotation", &Class::rotation, py_rvp::reference_internal,
            cls_doc.rotation.doc)
        .def("set_rotation",
            py::overload_cast<const RotationMatrix<T>&>(&Class::set_rotation),
            py::arg("R"), cls_doc.set_rotation.doc_1args_R)
        .def("set_rotation",
            py::overload_cast<const RollPitchYaw<T>&>(&Class::set_rotation),
            py::arg("rpy"), cls_doc.set_rotation.doc_1args_rpy)
        .def("set_rotation",
            py::overload_cast<const Eigen::Quaternion<T>&>(
                &Class::set_rotation),
            py::arg("quaternion"), cls_doc.set_rotation.doc_1args_quaternion)
        .def("set_rotation",
            py::overload_cast<const Eigen::AngleAxis<T>&>(&Class::set_rotation),
            py::arg("theta_lambda"),
            cls_doc.set_rotation.doc_1args_theta_lambda)
        .def("translation", &Class::translation,
            return_value_policy_for_scalar_type<T>(), cls_doc.translation.doc)
        .def("set_translation", &Class::set_translation, py::arg("p"),
            cls_doc.set_translation.doc)
        .def("GetAsMatrix4", &Class::GetAsMatrix4, cls_doc.GetAsMatrix4.doc)
        .def("GetAsMatrix34", &Class::GetAsMatrix34, cls_doc.GetAsMatrix34.doc)
        .def("GetAsIsometry3", &Class::GetAsIsometry3,
            cls_doc.GetAsIsometry3.doc)
        .def("SetIdentity", &Class::SetIdentity, cls_doc.SetIdentity.doc)
        .def("IsExactlyIdentity", &Class::IsExactlyIdentity,
            cls_doc.IsExactlyIdentity.doc)
        .def("IsNearlyIdentity", &Class::IsNearlyIdentity,
            py::arg("translation_tolerance"), cls_doc.IsNearlyIdentity.doc)
        .def("IsExactlyEqualTo", &Class::IsExactlyEqualTo, py::arg("other"),
            cls_doc.IsExactlyEqualTo.doc)
        .def("IsNearlyEqualTo", &Class::IsNearlyEqualTo, py::arg("other"),
            py::arg("tolerance"), cls_doc.IsNearlyEqualTo.doc)
        .def("inverse", &Class::inverse, cls_doc.inverse.doc)
        .def("InvertAndCompose", &Class::InvertAndCompose, py::arg("other"),
            cls_doc.InvertAndCompose.doc)
        .def("GetMaximumAbsoluteDifference",
            &Class::GetMaximumAbsoluteDifference, py::arg("other"),
            cls_doc.GetMaximumAbsoluteDifference.doc)
        .def("GetMaximumAbsoluteTranslationDifference",
            &Class::GetMaximumAbsoluteTranslationDifference, py::arg("other"),
            cls_doc.GetMaximumAbsoluteTranslationDifference.doc)
        .def(
            "multiply",
            [](const Class* self, const Class& other) { return *self * other; },
            py::arg("other"), cls_doc.operator_mul.doc_1args_other)
        .def(
            "multiply",
            [](const Class* self, const Vector3<T>& p_BoQ_B) {
              return *self * p_BoQ_B;
            },
            py::arg("p_BoQ_B"), cls_doc.operator_mul.doc_1args_p_BoQ_B)
        .def(
            "multiply",
            [](const Class* self, const Vector4<T>& vec_B) {
              return *self * vec_B;
            },
            py::arg("vec_B"), cls_doc.operator_mul.doc_1args_vec_B)
        .def(
            "multiply",
            [](const Class* self, const Matrix3X<T>& p_BoQ_B) {
              return *self * p_BoQ_B;
            },
            py::arg("p_BoQ_B"),
            cls_doc.operator_mul.doc_1args_constEigenMatrixBase)
        .def(py::pickle([](const Class& self) { return self.GetAsMatrix34(); },
            [](const Eigen::Matrix<T, 3, 4>& matrix) {
              return Class(matrix);
            }));
    cls.attr("multiply") = WrapToMatchInputShape(cls.attr("multiply"));
    cls.attr("__matmul__") = cls.attr("multiply");
    DefCopyAndDeepCopy(&cls);
    DefCast<T>(&cls, cls_doc.cast.doc);
    // N.B. The definition for `__repr__` is in `_math_extra.py`.

    AddValueInstantiation<Class>(m);

    // Some ports need `Value<std::vector<Class>>`.
    AddValueInstantiation<std::vector<Class>>(m);
  }
}

}  // namespace

void DefineMathRigidTransform(py::module m) {
  type_visit([m](auto dummy) { DoBind(m, dummy); }, CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
