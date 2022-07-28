#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/visualization/visualization_py.h"
#include "drake/visualization/colorize_depth_image.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineVisualizationColorize(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::visualization;
  constexpr auto& doc = pydrake_doc.drake.visualization;

  type_visit(
      [&](auto dummy) {
        using T = decltype(dummy);
        using Class = ColorizeDepthImage<T>;
        constexpr auto& cls_doc = doc.ColorizeDepthImage;
        DefineTemplateClassWithDefault<Class, systems::LeafSystem<T>>(
            m, "ColorizeDepthImage", GetPyParam<T>(), cls_doc.doc)
            .def(py::init<>(), cls_doc.ctor.doc_0args)
            .def(py::init<systems::sensors::PixelType>(),
                py::arg("depth_pixel_type"), cls_doc.ctor.doc_1args)
            .def_static("Calc",
                py::overload_cast<const systems::sensors::ImageDepth32F&,
                    systems::sensors::ImageRgba8U*>(&Class::Calc),
                py::arg("input"), py::arg("output"), cls_doc.Calc.doc)
            .def_static("Calc",
                py::overload_cast<const systems::sensors::ImageDepth16U&,
                    systems::sensors::ImageRgba8U*>(&Class::Calc),
                py::arg("input"), py::arg("output"), cls_doc.Calc.doc);
      },
      CommonScalarPack{});
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
