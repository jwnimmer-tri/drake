#include "drake/visualization/inertia_visualizer.h"

#include <memory>
#include <utility>

namespace drake {
namespace visualization {

using geometry::Box;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::Rgba;
using geometry::SceneGraph;
using math::RigidTransform;
using systems::Context;
using systems::DiagramBuilder;

template <typename T>
InertiaVisualizer<T>::InertiaVisualizer(
    const multibody::MultibodyPlant<T>& plant, SceneGraph<T>* scene_graph,
    InertiaVisualizerParams params)
    : params_{std::move(params)} {
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  source_id_ = scene_graph->RegisterSource("inertia_visualizer");

  // For all MbP bodies, except for those welded to the world ...
  const std::vector<const multibody::Body<T>*> world_bodies =
      plant.GetBodiesWeldedTo(plant.world_body());
  const int num_bodies = plant.num_bodies();
  for (multibody::BodyIndex i{0}; i < num_bodies; ++i) {
    bool welded = false;
    for (const auto* world_body : world_bodies) {
      if (world_body->index() == i) {
        welded = true;
        break;
      }
    }
    if (welded) {
      continue;
    }
    const multibody::Body<T>& body = plant.get_body(i);

    // Add a Bcm geometry frame.
    Item item;
    item.body = i;
    item.Bo_frame = plant.GetBodyFrameIdIfExists(i).value();
    item.Bcm_frame = scene_graph->RegisterFrame(
        source_id_, SceneGraph<T>::world_frame_id(),
        GeometryFrame{fmt::format(
            "InertiaVisualizer::{}::{}",
            plant.GetModelInstanceName(body.model_instance()), body.name())});

    // Add an illustration shape on Bcm.
    auto shape = std::make_unique<Box>(0.001, 0.001, 0.001);
    auto geom = std::make_unique<GeometryInstance>(
        RigidTransform<double>(), std::move(shape),
        fmt::format("$inertia({})", i));
    IllustrationProperties props;
    // FIXME Make this invisible by default.
    props.AddProperty("phong", "diffuse", Rgba{0.0, 0.0, 1.0, 0.2});
    geom->set_illustration_properties(std::move(props));
    item.geometry = scene_graph->RegisterGeometry(source_id_, item.Bcm_frame,
                                                  std::move(geom));
    items_.push_back(std::move(item));
  }

  // Update the geometry information to reflect the default inertia values
  // from the plant.
  UpdateItems(plant, *plant.CreateDefaultContext(), scene_graph);

  this->DeclareAbstractInputPort("plant_geometry_pose",
                                 Value<FramePoseVector<T>>());
  this->DeclareAbstractOutputPort("geometry_pose",
                                  &InertiaVisualizer<T>::CalcFramePoseOutput);
}

template <typename T>
const InertiaVisualizer<T>& InertiaVisualizer<T>::AddToBuilder(
    DiagramBuilder<T>* builder, const multibody::MultibodyPlant<T>& plant,
    SceneGraph<T>* scene_graph, InertiaVisualizerParams params) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DRAKE_THROW_UNLESS(scene_graph != nullptr);
  auto result = builder->template AddSystem<InertiaVisualizer<T>>(
      plant, scene_graph, std::move(params));
  builder->Connect(plant.get_geometry_poses_output_port(),
                   result->get_input_port());
  builder->Connect(result->get_output_port(),
                   scene_graph->get_source_pose_port(result->source_id_));
  return *result;
}

template <typename T>
InertiaVisualizer<T>::~InertiaVisualizer() = default;

template <typename T>
void InertiaVisualizer<T>::UpdateItems(
    const multibody::MultibodyPlant<T>& plant, const Context<T>& plant_context,
    SceneGraph<T>* scene_graph) {
  for (auto& item : items_) {
    // Interrogate the plant context for the inertia information.
    const multibody::Body<T>& body = plant.get_body(item.body);
    const double mass = ExtractDoubleOrThrow(body.get_mass(plant_context));
    // FIXME
    // const Vector3<T> com = body.CalcCenterOfMassInBodyFrame(plant_context);

    // Compute the equivalent illustration box.
    (void)(mass);                                  // FIXME
    const Eigen::Vector3d box_dim{0.1, 0.1, 0.1};  // FIXME
    const RigidTransform<double> X_BBcm;           // FIXME

    // Update the visualization.
    const Box box(box_dim);
    scene_graph->ChangeShape(source_id_, item.geometry, box);
    item.X_BBcm = X_BBcm;
  }
}

template <typename T>
void InertiaVisualizer<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  const auto& plant_poses =
      this->get_input_port().template Eval<FramePoseVector<T>>(context);

  poses->clear();
  for (const auto& item : items_) {
    const RigidTransform<T>& X_WBo = plant_poses.value(item.Bo_frame);
    const RigidTransform<T> X_WBcm = X_WBo * item.X_BBcm.template cast<T>();
    poses->set_value(item.Bcm_frame, X_WBcm);
  }
}

}  // namespace visualization
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::InertiaVisualizer)