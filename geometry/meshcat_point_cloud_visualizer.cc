#include "drake/geometry/meshcat_point_cloud_visualizer.h"

#include <memory>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/extract_double.h"
#include "drake/geometry/utilities.h"
#include "drake/perception/point_cloud.h"

namespace drake {
namespace geometry {

using math::RigidTransform;
using perception::PointCloud;
using systems::Context;
using systems::EventStatus;
using systems::SystemTypeTag;

// TODO(jwnimmer-tri) delete_on_initialization_event should be configurable.
template <typename T>
MeshcatPointCloudVisualizer<T>::MeshcatPointCloudVisualizer(
    std::shared_ptr<Meshcat> meshcat_in, std::string path,
    double publish_period)
    : MeshcatVisualizerBase<T>(
          SystemTypeTag<MeshcatPointCloudVisualizer>{}, std::move(meshcat_in),
          std::move(path), publish_period,
          /* publish_offset = */ 0.0,
          /* params.delete_on_initialization_event = */ false) {
  cloud_ =
      this->DeclareAbstractInputPort("cloud", Value<PointCloud>()).get_index();
  pose_ = this->DeclareAbstractInputPort("X_ParentCloud",
                                         Value<RigidTransform<T>>{})
              .get_index();
}

template <typename T>
template <typename U>
MeshcatPointCloudVisualizer<T>::MeshcatPointCloudVisualizer(
    const MeshcatPointCloudVisualizer<U>& other)
    : MeshcatPointCloudVisualizer(other.shared_meshcat(), other.prefix(),
                                  other.publish_period()) {
  set_point_size(other.point_size_);
  set_default_rgba(other.default_rgba_);
}

template <typename T>
EventStatus MeshcatPointCloudVisualizer<T>::DoOnPublish(
    double time, const Context<T>& context) const {
  // TODO(jwnimmer-tri) Pass the timestamp to meshcat.
  unused(time);

  const auto& cloud = cloud_input_port().template Eval<PointCloud>(context);
  meshcat().SetObject(prefix(), cloud, point_size_, default_rgba_);

  const RigidTransform<double> X_ParentCloud =
      pose_input_port().HasValue(context)
          ? internal::convert_to_double(
                pose_input_port().template Eval<RigidTransform<T>>(context))
          : RigidTransform<double>::Identity();
  meshcat().SetTransform(prefix(), X_ParentCloud);

  return EventStatus::Succeeded();
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatPointCloudVisualizer)
