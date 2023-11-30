#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/visualization/visualization_py.h"
#include "drake/visualization/lcm_image_stream_worker.h"

namespace drake {
namespace pydrake {
namespace internal {

using visualization::internal::LcmImageStreamWorker;

void DefineVisualizationLcmImageStreamWorker(py::module m) {
  using Class = LcmImageStreamWorker;
  py::class_<Class>(m, "_LcmImageStreamWorker")
      .def(py::init<std::string, std::string_view>(), py::arg("lcm_url"),
          py::arg("channel_regex"))
      .def("GetChannelNames", &Class::GetChannelNames)
      .def(
          "GetLatestPng",
          [](const Class& self, std::string_view channel_name) {
            std::shared_ptr<const std::vector<uint8_t>> png_data;
            int sequence{};
            std::tie(png_data, sequence) = self.GetLatestPng(channel_name);
            // Copy the data into a py::bytes.
            // TODO(jwnimmer-tri) Only copy when nothing has changed.
            return py::bytes(reinterpret_cast<const char*>(png_data->data()),
                png_data->size());
          },
          py::arg("channel_name"));
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
