#include "drake/geometry/meshcat_visualizer_base.h"

#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/extract_double.h"
#include "drake/common/unused.h"

namespace drake {
namespace geometry {

using systems::Context;
using systems::EventStatus;
using systems::LeafSystem;
using systems::SystemScalarConverter;

template <typename T>
MeshcatVisualizerBase<T>::MeshcatVisualizerBase(
    systems::SystemScalarConverter converter, std::shared_ptr<Meshcat> meshcat,
    std::string prefix, double publish_period, double publish_offset,
    bool delete_on_initialization_event)
    : LeafSystem<T>(std::move(converter)),
      meshcat_(std::move(meshcat)),
      prefix_(std::move(prefix)),
      publish_period_(publish_period),
      publish_offset_(publish_offset),
      delete_on_initialization_event_(delete_on_initialization_event) {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  DRAKE_THROW_UNLESS(publish_period_ >= 0.0);
  this->DeclareInitializationPublishEvent(
      &MeshcatVisualizerBase<T>::OnInitialize);
  this->DeclarePeriodicPublishEvent(publish_period_, publish_offset_,
                                    &MeshcatVisualizerBase<T>::OnPublish);
  this->DeclareForcedPublishEvent(&MeshcatVisualizerBase<T>::OnPublish);
}

template <typename T>
MeshcatVisualizerBase<T>::~MeshcatVisualizerBase() = default;

template <typename T>
void MeshcatVisualizerBase<T>::Delete() const {
  const EventStatus status = DoOnDelete();
  meshcat_->Delete(prefix_);
  DRAKE_THROW_UNLESS(status.severity() != EventStatus::kFailed);
}

template <typename T>
EventStatus MeshcatVisualizerBase<T>::DoOnDelete() const {
  // Derived classes are permitted to do nothing.
  return EventStatus::DidNothing();
}

template <typename T>
EventStatus MeshcatVisualizerBase<T>::OnInitialize(
    const Context<T>& context) const {
  EventStatus result = EventStatus::DidNothing();
  if (delete_on_initialization_event_) {
    const EventStatus delete_status = DoOnDelete();
    result.KeepMoreSevere(delete_status);
    meshcat_->Delete(prefix_);
  }
  const double time = ExtractDoubleOrThrow(context.get_time());
  const EventStatus init_status = DoOnInitialize(time, context);
  result.KeepMoreSevere(init_status);
  return result;
}

template <typename T>
EventStatus MeshcatVisualizerBase<T>::DoOnInitialize(
    double time, const Context<T>& context) const {
  // Derived classes are permitted to do nothing.
  unused(time, context);
  return EventStatus::DidNothing();
}

template <typename T>
EventStatus MeshcatVisualizerBase<T>::OnPublish(
    const Context<T>& context) const {
  const double time = ExtractDoubleOrThrow(context.get_time());
  return DoOnPublish(time, context);
}

template <typename T>
typename LeafSystem<T>::GraphvizFragment
MeshcatVisualizerBase<T>::DoGetGraphvizFragment(
    const typename LeafSystem<T>::GraphvizFragmentParams& params) const {
  typename LeafSystem<T>::GraphvizFragmentParams new_params{params};
  new_params.header_lines.push_back(fmt::format("path=/drake/{}", prefix_));
  typename LeafSystem<T>::GraphvizFragment result =
      LeafSystem<T>::DoGetGraphvizFragment(new_params);
  result.fragments.push_back(
      fmt::format("Meshcat [color=magenta];\n"
                  "{}:e -> Meshcat [style=dashed, color=magenta]\n",
                  new_params.node_id));
  return result;
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::MeshcatVisualizerBase)
