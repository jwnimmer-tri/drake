#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/multibody/multibody_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

namespace drake {
namespace pydrake {
namespace internal {

using geometry::SceneGraph;
using systems::LeafSystem;

// TODO(eric.cousineau): Bind additional scalar types.
using T = double;

void DefineMultibodyBenchmarks(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody::benchmarks::acrobot;
  constexpr auto& doc = pydrake_doc.drake.multibody.benchmarks.acrobot;

  py::class_<AcrobotParameters>(
      m, "AcrobotParameters", doc.AcrobotParameters.doc)
      .def(py::init(), doc.AcrobotParameters.doc);

  m.def("MakeAcrobotPlant",
      py::overload_cast<const AcrobotParameters&, bool, SceneGraph<double>*>(
          &MakeAcrobotPlant),
      py::arg("default_parameters"), py::arg("finalize"),
      py::arg("scene_graph") = nullptr, doc.MakeAcrobotPlant.doc);
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
