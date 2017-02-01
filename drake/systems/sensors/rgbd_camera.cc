#include "drake/systems/sensors/rgbd_camera.h"

#include <Eigen/Dense>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkImageShiftScale.h>
#include <vtkNew.h>
#include <vtkOBJReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPNGReader.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkWindowToImageFilter.h>

#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/vtk_util.h"

// TODO(kunimatsu.hashimoto) Refactor RenderingWorld out from RGBDCamera,
// so that other vtk powered sensor simulators can share the RenderingWorld
// without duplicating it.

namespace drake {
namespace systems {
namespace sensors {
namespace {
const double kRadToDeg = 57.29577951308232;
const double kDegToRad = 1. / kRadToDeg;

const int kImageWidth = 640;  // in pixels
const int kImageHeight = 480;  // in pixels
const int kImageChannel = 4;
const double kClippingPlaneNear = 0.5;  // in meters
const double kClippingPlaneFar = 5.0;  // in meters
const double kVerticalFovDeg = 45.;
const double kFrameRateHz = 30.;

// TODO(kunimatsu.hashimoto) Calculates this in vertex shader.
// For Zbuffer value conversion
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = -kA * kClippingPlaneNear;
double ConvertZbufferToMeters(float z_buffer_value) {
  return kB / (z_buffer_value - kA);
}

std::string RemoveFileExtention(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_ASSERT(false);
  }
  return filepath.substr(0, last_dot);
}

}  // anonymous namespace

class RGBDCamera::Impl {
 public:
  Impl(const RigidBodyTree<double>& tree,const RigidBodyFrame<double>& frame,
       bool show_window);

  Impl(const RigidBodyTree<double>& tree, const Eigen::Vector3d& position,
       const Eigen::Vector3d& orientation, bool show_window);

  ~Impl() {}

  const RigidBodyTree<double>& get_tree() const { return tree_; }

  const CameraInfo& get_camera_info(const RGBDCamera::CameraType type) const {
    if ((type != RGBDCamera::CameraType::kColor) &&
        (type != RGBDCamera::CameraType::kDepth)) {
      throw std::runtime_error("Unknown CameraType specified");
    }
    return camera_info_array_[type];
  }

  void DoCalcOutput(const BasicVector<double>& input_vector,
                    systems::SystemOutput<double>* output) const;

 private:
  void CreateRenderingWorld();

  void UpdateModelFrames(const VectorBase<double>& vector_base) const;

  void UpdateRender() const;

  void SetCameraPoseAtWorld(const Eigen::Vector3d& position,
                            const Eigen::Vector4d& axis_angle) const;

  const RigidBodyTree<double>& tree_;
  std::array<CameraInfo, 2> camera_info_array_;
  std::map<int, vtkSmartPointer<vtkActor>> id_object_pairs_;
  vtkNew<vtkRenderer> renderer_;
  vtkNew<vtkRenderWindow> render_window_;
  vtkNew<vtkWindowToImageFilter> depth_buffer_;
  vtkNew<vtkWindowToImageFilter> color_buffer_;
};


RGBDCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       bool show_window)
    : tree_(tree), camera_info_array_{
          CameraInfo(static_cast<uint32_t>(kImageWidth),
                     static_cast<uint32_t>(kImageHeight),
                     kVerticalFovDeg * kDegToRad),
          CameraInfo(static_cast<uint32_t>(kImageWidth),
                     static_cast<uint32_t>(kImageHeight),
                     kVerticalFovDeg * kDegToRad)} {
  if (!show_window) {
    render_window_->SetOffScreenRendering(1);
  }

  CreateRenderingWorld();

  // Setting camera
  vtkNew<vtkCamera> camera;
  // TODO(kunimatsu.hashimoto) Add support for the arbitrary view angle.
  camera->SetViewAngle(kVerticalFovDeg);

  // TODO(kunimatsu.hashimoto) Lets rgb and depth cameras have independent
  // clipping planes.
  // TODO(kunimatsu.hashimoto) Add support for the arbitrary clipping planes.
  camera->SetClippingRange(kClippingPlaneNear, kClippingPlaneFar);
  renderer_->SetActiveCamera(camera.GetPointer());

  // Set camera pose
  renderer_->GetActiveCamera()->Yaw(orientation[2]);
  renderer_->GetActiveCamera()->Pitch(orientation[1]);
  renderer_->GetActiveCamera()->Roll(orientation[0]);
  renderer_->GetActiveCamera()->SetPosition(position[0],
                                            position[1],
                                            position[2]);

  // TODO(kunimatsu.hashimoto) Add support for image of arbitrary size.
  render_window_->SetSize(kImageWidth, kImageHeight);
  render_window_->AddRenderer(renderer_.GetPointer());

  color_buffer_->SetInput(render_window_.GetPointer());
  color_buffer_->SetMagnification(1);
  color_buffer_->SetInputBufferTypeToRGBA();
  color_buffer_->Update();

  depth_buffer_->SetInput(render_window_.GetPointer());
  depth_buffer_->SetMagnification(1);
  depth_buffer_->SetInputBufferTypeToZBuffer();
  depth_buffer_->Update();
}

RGBDCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       bool show_window) : tree_(tree), camera_info_array_{
          CameraInfo(static_cast<uint32_t>(kImageWidth),
                     static_cast<uint32_t>(kImageHeight),
                     kVerticalFovDeg * kDegToRad),
          CameraInfo(static_cast<uint32_t>(kImageWidth),
                     static_cast<uint32_t>(kImageHeight),
                     kVerticalFovDeg * kDegToRad)} {
  // TODO(kunimatsu) Implement this
  std::runtime_error("Not implemented");
}

void RGBDCamera::Impl::CreateRenderingWorld() {
  for (const auto& body : tree_.bodies) {
    int const model_id = body->get_model_instance_id();
    // Assuming that a rigid body owns only a visual element.
    for (const auto& visual : body->get_visual_elements()) {

      auto t = visual.getWorldTransform().translation();
      auto axis_angle = drake::math::rotmat2axis(
          visual.getWorldTransform().linear());

      vtkNew<vtkTransform> vtk_transform;
      // The order should be Translate first, and then RotateWXYZ.
      vtk_transform->Translate(t[0], t[1], t[2]);
      vtk_transform->RotateWXYZ(axis_angle[3] * kRadToDeg ,
                                axis_angle[0], axis_angle[1], axis_angle[2]);

      vtkNew<vtkActor> actor;
      vtkNew<vtkPolyDataMapper> mapper;
      bool shape_matched = true;
      const DrakeShapes::Geometry& geometry = visual.getGeometry();
      switch (visual.getShape()) {
        case DrakeShapes::BOX: {
          auto box = dynamic_cast<const DrakeShapes::Box&>(geometry);
          vtkNew<vtkCubeSource> vtk_cube;
          vtk_cube->SetXLength(box.size(0));
          vtk_cube->SetYLength(box.size(1));
          vtk_cube->SetZLength(box.size(2));

          mapper->SetInputConnection(vtk_cube->GetOutputPort());
          break;
        }
        case DrakeShapes::SPHERE: {
          auto sphere = dynamic_cast<const DrakeShapes::Sphere&>(geometry);
          vtkNew<vtkSphereSource> vtk_sphere;
          vtk_sphere->SetRadius(sphere.radius);
          vtk_sphere->SetThetaResolution(50);
          vtk_sphere->SetPhiResolution(50);

          mapper->SetInputConnection(vtk_sphere->GetOutputPort());
          break;
        }
        case DrakeShapes::CYLINDER: {
          auto cylinder = dynamic_cast<const DrakeShapes::Cylinder&>(geometry);
          vtkNew<vtkCylinderSource> vtk_cylinder;
          vtk_cylinder->SetHeight(cylinder.length);
          vtk_cylinder->SetRadius(cylinder.radius);
          vtk_cylinder->SetResolution(50);

          mapper->SetInputConnection(vtk_cylinder->GetOutputPort());
          break;
        }
        case DrakeShapes::MESH: {
          auto m = dynamic_cast<const DrakeShapes::Mesh&>(geometry);

          // TODO(kunimatsu.hashimoto) Add support for other file formats.
          vtkNew<vtkOBJReader> mesh_reader;
          mesh_reader->SetFileName(m.resolved_filename_.c_str());
          mesh_reader->Update();

          // TODO(kunimatsu.hashimoto) Add support for other file formats.
          vtkNew<vtkPNGReader> texture_reader;
          texture_reader->SetFileName(std::string(RemoveFileExtention(
              m.resolved_filename_.c_str()) + ".png").c_str());
          texture_reader->Update();

          vtkNew<vtkTexture> texture;
          texture->SetInputConnection(texture_reader->GetOutputPort());
          texture->InterpolateOn();

          mapper->SetInputConnection(mesh_reader->GetOutputPort());
          actor->SetTexture(texture.GetPointer());
          break;
        }
        case DrakeShapes::CAPSULE: {
          // TODO(kunimatsu.hashimoto) Implement this when needed.
          shape_matched = false;
          break;
        }
        default: {
          shape_matched = false;
          break;
        }
      }

      // Register actors
      if (shape_matched) {
        actor->SetMapper(mapper.GetPointer());
        actor->SetUserTransform(vtk_transform.GetPointer());
        id_object_pairs_[model_id] =
            vtkSmartPointer<vtkActor>(actor.GetPointer());
        renderer_->AddActor(actor.GetPointer());
      }
    }
  }

  // Adds a flat terrain.
  unsigned char plane_color[3] = {255, 229, 204};
  vtkSmartPointer<vtkPolyData> plane = CreateFlatTerrain(100., plane_color);

  vtkNew<vtkPolyDataMapper> mapper;
#if VTK_MAJOR_VERSION <= 5
  mapper->SetInputConnection(plane->GetProducerPort());
#else
  mapper->SetInputData(plane);
#endif

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper.GetPointer());
  renderer_->AddActor(actor.GetPointer());
}


void RGBDCamera::Impl::UpdateModelFrames(
    const VectorBase<double>& vector_base) const {
  const Eigen::VectorXd q = vector_base.CopyToVector().head(
      tree_.get_num_positions());

  KinematicsCache<double> cache = tree_.doKinematics(q);

  // TODO(kunimatsu-tri) Update camera frame by calling SetCameraPoseAtWorld

  for (const auto& body : tree_.bodies) {
    auto transform = tree_.relativeTransform(cache, 0, body->get_body_index());
    auto translation = transform.translation();
    auto axis_angle = drake::math::rotmat2axis(transform.linear());

    vtkNew<vtkTransform> vtk_transform;
    // The order should be Translate first, and then RotateWXYZ.
    vtk_transform->Translate(translation(0), translation(1), translation(2));
    vtk_transform->RotateWXYZ(axis_angle[3] * kRadToDeg ,
                              axis_angle[0], axis_angle[1], axis_angle[2]);

    auto& actor = id_object_pairs_.at(body->get_model_instance_id());
    actor->SetUserTransform(vtk_transform.GetPointer());
  }
}

void RGBDCamera::Impl::UpdateRender() const {
  render_window_->Render();
  color_buffer_->Modified();
  color_buffer_->Update();
  depth_buffer_->Modified();
  depth_buffer_->Update();
}

void RGBDCamera::Impl::DoCalcOutput(
    const BasicVector<double>& input_vector,
    systems::SystemOutput<double>* output) const {
  UpdateModelFrames(input_vector);

  UpdateRender();

  // Outputs the image data
  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  drake::systems::sensors::Image<uint8_t>& image =
      mutable_data->GetMutableValue<
        drake::systems::sensors::Image<uint8_t>>();

  systems::AbstractValue* mutable_data_d = output->GetMutableData(1);
  drake::systems::sensors::Image<float>& depth_image =
      mutable_data_d->GetMutableValue<
        drake::systems::sensors::Image<float>>();

  for (uint32_t r = 0; r < kImageHeight; ++r) {
    for (uint32_t c = 0; c < kImageWidth; ++c) {
      // Makes image upside down
      const uint32_t kHeight = kImageHeight - r - 1;

      // Color image
      void* color_ptr = color_buffer_->GetOutput()->GetScalarPointer(c,r,0);
      // Convert RGBA to BGRA
      image.at(c, kHeight)[0] = *(static_cast<uint8_t*>(color_ptr) + 2);  // B
      image.at(c, kHeight)[1] = *(static_cast<uint8_t*>(color_ptr) + 1);  // G
      image.at(c, kHeight)[2] = *(static_cast<uint8_t*>(color_ptr) + 0);  // R
      image.at(c, kHeight)[3] = *(static_cast<uint8_t*>(color_ptr) + 3);  // A

      // Depth image
      float depth_value = *static_cast<float*>(
          depth_buffer_->GetOutput()->GetScalarPointer(c,r,0));
      if (depth_value == 1.0) {
        depth_image.at(c, kHeight)[0] = 0.;
      } else {
        depth_image.at(c, kHeight)[0] = ConvertZbufferToMeters(depth_value);
      }
    }
  }
}

void RGBDCamera::Impl::SetCameraPoseAtWorld(
    const Eigen::Vector3d& position, const Eigen::Vector4d& axis_angle) const {
  // First, reset camera pose.
  vtkNew<vtkTransform> current_pose;
  current_pose->SetMatrix(
      renderer_->GetActiveCamera()->GetViewTransformMatrix());
  renderer_->GetActiveCamera()->ApplyTransform(current_pose.GetPointer());

  // Next, set camera pose with respect to the world frame.
  vtkNew<vtkTransform> new_pose;
  // Note: The order should be RotateWXYZ first, then Translate.
  new_pose->RotateWXYZ(axis_angle[3] * kRadToDeg ,
                       axis_angle[0], axis_angle[1], axis_angle[2]);
  new_pose->Translate(position[0], position[1], position[2]);
  renderer_->GetActiveCamera()->ApplyTransform(new_pose.GetPointer());
}


RGBDCamera::RGBDCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& orientation,
                       bool show_window)
    : impl_(new RGBDCamera::Impl(tree, position, orientation, show_window)) {
  set_name(name);
  const int vec_num =  tree.get_num_positions() + tree.get_num_velocities();
  input_port_index_ = this->DeclareInputPort(
      systems::kVectorValued, vec_num).get_index();

  this->DeclareAbstractOutputPort();
  this->DeclareAbstractOutputPort();
}

RGBDCamera::RGBDCamera(const std::string& name,
                       const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       bool show_window)
    : impl_(new RGBDCamera::Impl(tree, frame, show_window)) {
  set_name(name);
  const int vec_num =  tree.get_num_positions() + tree.get_num_velocities();
  input_port_index_ = this->DeclareInputPort(
      systems::kVectorValued, vec_num).get_index();

  this->DeclareAbstractOutputPort();
  this->DeclareAbstractOutputPort();
}


RGBDCamera::~RGBDCamera() {}

/*
const InputPortDescriptor<double>&
RGBDCamera::get_rigid_body_tree_state_input_port() const {
  return this->get_input_port(input_port_index_);
}

const OutputPortDescriptor<double>&
RGBDCamera::get_sensor_state_output_port() const {
  return System<double>::get_output_port(output_port_index_);
}
*/
std::unique_ptr<SystemOutput<double>> RGBDCamera::AllocateOutput(
    const Context<double>& context) const {
  auto output = std::make_unique<systems::LeafSystemOutput<double>>();

  sensors::Image<uint8_t> color_image(kImageWidth, kImageHeight, kImageChannel);
  output->add_port(
      std::make_unique<systems::Value<sensors::Image<uint8_t>>>(color_image));

  sensors::Image<float> depth_image(kImageWidth, kImageHeight, 1);
  output->add_port(
      std::make_unique<systems::Value<sensors::Image<float>>>(depth_image));

  return std::unique_ptr<SystemOutput<double>>(output.release());
}

void RGBDCamera::DoCalcOutput(const systems::Context<double>& context,
                              systems::SystemOutput<double>* output) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));
  const double kCurrentTime = context.get_time();
  // TODO(kunimatsu-tri) Add support for the arbitrary frame rate
  if (kCurrentTime == 0. ||
      (kCurrentTime - previous_output_time_ < 1. / kFrameRateHz)) {
    return;
  }
  previous_output_time_ = kCurrentTime;

  const BasicVector<double>* input_vector =
      this->EvalVectorInput(context, input_port_index_);

  impl_->DoCalcOutput(*input_vector, output);
}

const RigidBodyTree<double>& RGBDCamera::get_tree() const {
  return impl_->get_tree();
}

const CameraInfo& RGBDCamera::get_camera_info(const CameraType type) const {
  return impl_->get_camera_info(type);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
