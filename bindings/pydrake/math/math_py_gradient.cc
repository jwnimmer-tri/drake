#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/math/math_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/fmt_ostream.h"
#include "drake/math/compute_numerical_gradient.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineMathGradient(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::math;
  constexpr auto& doc = pydrake_doc.drake.math;

  {
    using Class = NumericalGradientMethod;
    constexpr auto& cls_doc = doc.NumericalGradientMethod;
    py::enum_<Class>(m, "NumericalGradientMethod", cls_doc.doc)
        .value("kForward", Class::kForward, cls_doc.kForward.doc)
        .value("kBackward", Class::kBackward, cls_doc.kBackward.doc)
        .value("kCentral", Class::kCentral, cls_doc.kCentral.doc);
  }

  {
    using Class = NumericalGradientOption;
    constexpr auto& cls_doc = doc.NumericalGradientOption;
    py::class_<Class>(m, "NumericalGradientOption", cls_doc.doc)
        .def(py::init<NumericalGradientMethod, double>(), py::arg("method"),
            py::arg("function_accuracy") = 1E-15, cls_doc.ctor.doc)
        .def("NumericalGradientMethod", &Class::method, cls_doc.method.doc)
        .def("perturbation_size", &Class::perturbation_size,
            cls_doc.perturbation_size.doc)
        .def(
            "__repr__", [](const NumericalGradientOption& self) -> std::string {
              py::object method = py::cast(self.method());
              // This is a minimal implementation that serves to avoid
              // displaying memory addresses in pydrake docs and help strings.
              // In the future, we should enhance this to display all of the
              // information.
              return fmt::format("<NumericalGradientOption({})>",
                  fmt_streamed(py::repr(method)));
            });
  }

  m.def(
      "ComputeNumericalGradient",
      [](std::function<Eigen::VectorXd(const Eigen::VectorXd&)> calc_func,
          const Eigen::VectorXd& x, const NumericalGradientOption& option) {
        std::function<void(const Eigen::VectorXd&, Eigen::VectorXd*)>
            calc_func_no_return =
                [&calc_func](const Eigen::VectorXd& x_val, Eigen::VectorXd* y) {
                  *y = calc_func(x_val);
                };
        return ComputeNumericalGradient(calc_func_no_return, x, option);
      },
      py::arg("calc_func"), py::arg("x"),
      py::arg("option") =
          NumericalGradientOption(NumericalGradientMethod::kForward),
      doc.ComputeNumericalGradient.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
