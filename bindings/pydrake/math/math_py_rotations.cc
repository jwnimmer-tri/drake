#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/math/random_rotation.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

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

  // Due to circular dependencies between RotationMatrix and RollPitchYaw, we
  // need to declare one of the two first, then bind the other, then go back
  // and bind the first one.
  auto roll_pitch_yaw_cls = DefineTemplateClassWithDefault<RollPitchYaw<T>>(
      m, "RollPitchYaw", param, doc.RollPitchYaw.doc);

  {
    using Class = RotationMatrix<T>;
    constexpr auto& cls_doc = doc.RotationMatrix;

    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "RotationMatrix", param, cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc_0args)
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<const Matrix3<T>&>(), py::arg("R"),
            cls_doc.ctor.doc_1args_R)
        .def(py::init<Eigen::Quaternion<T>>(), py::arg("quaternion"),
            cls_doc.ctor.doc_1args_quaternion)
        .def(py::init<const Eigen::AngleAxis<T>&>(), py::arg("theta_lambda"),
            cls_doc.ctor.doc_1args_theta_lambda)
        .def(py::init<const RollPitchYaw<T>&>(), py::arg("rpy"),
            cls_doc.ctor.doc_1args_rpy)
        .def_static("MakeXRotation", &Class::MakeXRotation, py::arg("theta"),
            cls_doc.MakeXRotation.doc)
        .def_static("MakeYRotation", &Class::MakeYRotation, py::arg("theta"),
            cls_doc.MakeYRotation.doc)
        .def_static("MakeZRotation", &Class::MakeZRotation, py::arg("theta"),
            cls_doc.MakeZRotation.doc)
        .def_static("MakeFromOneVector", &Class::MakeFromOneVector,
            py::arg("b_A"), py::arg("axis_index"),
            cls_doc.MakeFromOneVector.doc)
        .def_static("Identity", &Class::Identity, cls_doc.Identity.doc)
        .def("set", &Class::set, py::arg("R"), cls_doc.set.doc)
        .def("inverse", &Class::inverse, cls_doc.inverse.doc)
        .def("InvertAndCompose", &Class::InvertAndCompose, py::arg("other"),
            cls_doc.InvertAndCompose.doc)
        .def("transpose", &Class::transpose, cls_doc.transpose.doc)
        .def("matrix", &Class::matrix, cls_doc.matrix.doc)
        .def("row", &Class::row, py::arg("index"), cls_doc.row.doc)
        .def("col", &Class::col, py::arg("index"), cls_doc.col.doc)
        .def(
            "multiply",
            [](const Class& self, const Class& other) { return self * other; },
            py::arg("other"), cls_doc.operator_mul.doc_1args_other)
        .def(
            "multiply",
            [](const Class& self, const Vector3<T>& v_B) { return self * v_B; },
            py::arg("v_B"), cls_doc.operator_mul.doc_1args_v_B)
        .def(
            "multiply",
            [](const Class& self, const Matrix3X<T>& v_B) {
              return self * v_B;
            },
            py::arg("v_B"), cls_doc.operator_mul.doc_1args_constEigenMatrixBase)
        .def("IsValid", overload_cast_explicit<boolean<T>>(&Class::IsValid),
            cls_doc.IsValid.doc_0args)
        .def("IsExactlyIdentity", &Class::IsExactlyIdentity,
            cls_doc.IsExactlyIdentity.doc)
        .def("IsNearlyIdentity", &Class::IsNearlyIdentity,
            py::arg("tolerance") =
                Class::get_internal_tolerance_for_orthonormality(),
            cls_doc.IsNearlyIdentity.doc)
        // Does not return the quality_factor
        .def_static(
            "ProjectToRotationMatrix",
            [](const Matrix3<T>& M) {
              return RotationMatrix<T>::ProjectToRotationMatrix(M);
            },
            py::arg("M"), cls_doc.ProjectToRotationMatrix.doc)
        .def("ToRollPitchYaw", &Class::ToRollPitchYaw,
            cls_doc.ToRollPitchYaw.doc)
        .def("ToQuaternion",
            overload_cast_explicit<Eigen::Quaternion<T>>(&Class::ToQuaternion),
            cls_doc.ToQuaternion.doc_0args)
        .def("ToAngleAxis", &Class::ToAngleAxis, cls_doc.ToAngleAxis.doc)
        .def(py::pickle([](const Class& self) { return self.matrix(); },
            [](const Matrix3<T>& matrix) { return Class(matrix); }));
    cls.attr("multiply") = WrapToMatchInputShape(cls.attr("multiply"));
    cls.attr("__matmul__") = cls.attr("multiply");
    DefCopyAndDeepCopy(&cls);
    DefCast<T>(&cls, cls_doc.cast.doc);
    // N.B. The definition for `__repr__` is in `_math_extra.py`.

    AddValueInstantiation<Class>(m);

    // Some ports need `Value<std::vector<Class>>`.
    AddValueInstantiation<std::vector<Class>>(m);
  }

  {
    using Class = RollPitchYaw<T>;
    constexpr auto& cls_doc = doc.RollPitchYaw;
    auto& cls = roll_pitch_yaw_cls;
    cls  // BR
        .def(py::init<const Class&>(), py::arg("other"))
        .def(py::init<const Vector3<T>>(), py::arg("rpy"),
            cls_doc.ctor.doc_1args_rpy)
        .def(py::init<const T&, const T&, const T&>(), py::arg("roll"),
            py::arg("pitch"), py::arg("yaw"),
            cls_doc.ctor.doc_3args_roll_pitch_yaw)
        .def(py::init<const RotationMatrix<T>&>(), py::arg("R"),
            cls_doc.ctor.doc_1args_R)
        .def(py::init<const Eigen::Quaternion<T>&>(), py::arg("quaternion"),
            cls_doc.ctor.doc_1args_quaternion)
        .def(py::init([](const Matrix3<T>& matrix) {
          return Class(RotationMatrix<T>(matrix));
        }),
            py::arg("matrix"),
            "Construct from raw rotation matrix. See RotationMatrix overload "
            "for more information.")
        .def("vector", &Class::vector, cls_doc.vector.doc)
        .def("roll_angle", &Class::roll_angle, cls_doc.roll_angle.doc)
        .def("pitch_angle", &Class::pitch_angle, cls_doc.pitch_angle.doc)
        .def("yaw_angle", &Class::yaw_angle, cls_doc.yaw_angle.doc)
        .def("ToQuaternion", &Class::ToQuaternion, cls_doc.ToQuaternion.doc)
        .def("ToRotationMatrix", &Class::ToRotationMatrix,
            cls_doc.ToRotationMatrix.doc)
        .def("CalcRotationMatrixDt", &Class::CalcRotationMatrixDt,
            py::arg("rpyDt"), cls_doc.CalcRotationMatrixDt.doc)
        .def("CalcAngularVelocityInParentFromRpyDt",
            &Class::CalcAngularVelocityInParentFromRpyDt, py::arg("rpyDt"),
            cls_doc.CalcAngularVelocityInParentFromRpyDt.doc)
        .def("CalcAngularVelocityInChildFromRpyDt",
            &Class::CalcAngularVelocityInChildFromRpyDt, py::arg("rpyDt"),
            cls_doc.CalcAngularVelocityInChildFromRpyDt.doc)
        .def("CalcRpyDtFromAngularVelocityInParent",
            &Class::CalcRpyDtFromAngularVelocityInParent, py::arg("w_AD_A"),
            cls_doc.CalcRpyDtFromAngularVelocityInParent.doc)
        .def("CalcRpyDDtFromRpyDtAndAngularAccelInParent",
            &Class::CalcRpyDDtFromRpyDtAndAngularAccelInParent,
            py::arg("rpyDt"), py::arg("alpha_AD_A"),
            cls_doc.CalcRpyDDtFromRpyDtAndAngularAccelInParent.doc)
        .def("CalcRpyDDtFromAngularAccelInChild",
            &Class::CalcRpyDDtFromAngularAccelInChild, py::arg("rpyDt"),
            py::arg("alpha_AD_D"),
            cls_doc.CalcRpyDDtFromAngularAccelInChild.doc)
        .def(py::pickle([](const Class& self) { return self.vector(); },
            [](const Vector3<T>& rpy) { return Class(rpy); }));
    DefCopyAndDeepCopy(&cls);
    // N.B. The definition for `__repr__` is in `_math_extra.py`.
    // N.B. `RollPitchYaw::cast` is not defined in C++.
  }

  // TODO(eric.cousineau): Bind remaining classes for all available scalar
  // types.
  if constexpr (std::is_same_v<T, double>) {
    // Random Rotations
    m  // BR
        .def("UniformlyRandomQuaternion",
            overload_cast_explicit<Eigen::Quaternion<T>, RandomGenerator*>(
                &UniformlyRandomQuaternion),
            py::arg("generator"), doc.UniformlyRandomQuaternion.doc)
        .def("UniformlyRandomAngleAxis",
            overload_cast_explicit<Eigen::AngleAxis<T>, RandomGenerator*>(
                &UniformlyRandomAngleAxis),
            py::arg("generator"), doc.UniformlyRandomAngleAxis.doc)
        .def("UniformlyRandomRotationMatrix",
            overload_cast_explicit<RotationMatrix<T>, RandomGenerator*>(
                &UniformlyRandomRotationMatrix),
            py::arg("generator"), doc.UniformlyRandomRotationMatrix.doc)
        .def("UniformlyRandomRPY",
            overload_cast_explicit<Vector3<T>, RandomGenerator*>(
                &UniformlyRandomRPY),
            py::arg("generator"), doc.UniformlyRandomRPY.doc);
  }
}

}  // namespace

void DefineMathRotations(py::module m) {
  type_visit([m](auto dummy) { DoBind(m, dummy); }, CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
