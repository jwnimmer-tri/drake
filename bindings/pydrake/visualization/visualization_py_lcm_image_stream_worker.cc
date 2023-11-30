#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/visualization/visualization_py.h"
#include "drake/visualization/lcm_image_stream_worker.h"

namespace drake {
namespace pydrake {
namespace internal {
namespace {

#if 0
    // Copy the data into a py::bytes.
    return py::bytes(
        reinterpret_cast<const char*>(png_data.data()), png_data.size());
#endif

}  // namespace

void DefineVisualizationLcmImageWorker(py::module m) {

#if 0  
class LcmImageStreamWorker final {
  LcmImageStreamWorker(std::string lcm_url, std::string_view channel_regex);
  std::vector<std::string> GetChannelNames() const;
  std::pair<std::shared_ptr<const std::vector<uint8_t>>, int> GetLatestPng(
      std::string_view channel_name) const;
#endif

    using Class = LcmImageWorker;
    py::class_<Class>(m, "_LcmImageWorker")
        .def(py::init<const std::string&>(), py::arg("channel_name"))
        .def("PopNext", &Class::PopNext);
  
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
