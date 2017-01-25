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

#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/vtk_util.h"


// TODO(kunimatsu.hashimoto) Refactor RenderingWorld out from RGBDCamera,
// so that other sensors can share the RenderingWorld.

namespace drake {
namespace systems {
namespace sensors {
namespace {
const double kRadToDeg = 57.29577951308232;

const int kPortIndex = 0;

const uint32_t kImageWidth = 640;  // in pixels
const uint32_t kImageHeight = 480;  // in pixels
const uint32_t kNumPixels = kImageWidth * kImageHeight;
const double kClippingPlaneNear = 0.5;  // in meters
const double kClippingPlaneFar = 5.0;  // in meters
const double kVerticalFovDeg = 45.;  // in degrees

// For Zbuffer value conversion
const double kA = kClippingPlaneFar / (kClippingPlaneFar - kClippingPlaneNear);
const double kB = kClippingPlaneFar * kClippingPlaneNear / (
    kClippingPlaneNear - kClippingPlaneFar);

std::string RemoveFileExtention(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    DRAKE_ASSERT(false);
  }
  return filepath.substr(0, last_dot);
}

// TODO(kunimatsu.hashimoto) Calculates this in vertex shader.
// double ConvertZbufferToMeters(float z_buffer_value) {
//   return kB / (static_cast<float>(z_buffer_value) - kA);
// }

}  // anonymous namespace

class RGBDCamera::Impl {
 public:
  Impl(const RigidBodyTree<double>& tree,const RigidBodyFrame<double>& frame,
       bool show_window);

  ~Impl() {}

  void CreateRenderingWorld();

  void UpdateModelFrames(const VectorBase<double>& vector_base) const;

  void DoCalcOutput(const BasicVector<double>& input_vector,
                    systems::SystemOutput<double>* output) const;

  const RigidBodyTree<double>& tree_;
  const RigidBodyFrame<double>& frame_;
  std::map<int, vtkSmartPointer<vtkActor>> id_object_pairs_;
  vtkNew<vtkRenderer> renderer_;
  vtkNew<vtkRenderWindow> render_window_;
  vtkNew<vtkWindowToImageFilter> depth_buffer_;
  vtkNew<vtkWindowToImageFilter> color_buffer_;

 private:
  void SetCameraPoseAtWorld(const Eigen::Vector3d& position,
                            const Eigen::Vector4d& axis_angle) const;
};

// RGBDCamera::Impl::Impl(const Eigen::Vector3d& position,
//                        const Eigen::Vector3d& orientation,
//                        const RigidBodyTree<double>& tree,
//                        bool show_window)

RGBDCamera::Impl::Impl(const RigidBodyTree<double>& tree,
                       const RigidBodyFrame<double>& frame,
                       bool show_window)
    : tree_(tree), frame_(frame) {
  if (show_window) {
    render_window_->SetOffScreenRendering(1);
    // render_window_->SetOffScreenRenderingOn();
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
        // would prefer to do this through virtual methods, but don't want to
        // introduce any vtk dependency on the Geometry classes
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
  auto camera_transform = tree_.relativeTransform(
      cache, 0, frame_.get_frame_index());
  auto camera_position = camera_transform.translation();
  auto camera_axis_angle = drake::math::rotmat2axis(camera_transform.linear());

  SetCameraPoseAtWorld(camera_position, camera_axis_angle);

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

void RGBDCamera::Impl::DoCalcOutput(
    const BasicVector<double>& input_vector,
    systems::SystemOutput<double>* output) const {
  UpdateModelFrames(input_vector);
  render_window_->Render();

  // Outputs the image data
  // TODO(kunimatsu.hashimoto) Defines Image class.
  // Eigen::VectorBlock<VectorX<double>> image =
  //     output->GetMutableVectorData(0)->get_mutable_value();

  // TODO Do the same thing for depth.
  systems::AbstractValue* mutable_data = output->GetMutableData(0);
  drake::systems::sensors::Image<uint8_t, 4>& image =
      mutable_data->GetMutableValue<
        drake::systems::sensors::Image<uint8_t, 4>>();

  for (uint32_t r = 0; r < kImageHeight; ++r) {
    // const uint32_t offset = r * kImageWidth;
    for (uint32_t c = 0; c < kImageWidth; ++c) {
      // Color image
      // TODO kunimatsu convert from RGBA to BGRA
      void* ptr = color_buffer_->GetOutput()->GetScalarPointer(c,r,0);
      // image(offset + c) = static_cast<double>(
      //     *static_cast<uint32_t*>(ptr));
      image.at(c, r)[0] = *static_cast<uint8_t*>(ptr);
      image.at(c, r)[1] = *(static_cast<uint8_t*>(ptr) + 1);
      image.at(c, r)[2] = *(static_cast<uint8_t*>(ptr) + 2);
      image.at(c, r)[3] = *(static_cast<uint8_t*>(ptr) + 3);

      // float depth = *static_cast<float*>(
      //     depth_buffer_->GetOutput()->GetScalarPointer(c,r,0));
      // if (depth == 1.0) {
      //   image(offset + c + kNumPixels) = 0.;
      // } else {
      //   image(offset + c + kNumPixels) = ConvertZbufferToMeters(depth);
      // }
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
                       const RigidBodyFrame<double>& frame,
                       bool show_window)
    : impl_(new RGBDCamera::Impl(tree, frame, show_window)) {
  set_name(name);
  const int vec_num =  tree.get_num_positions() + tree.get_num_velocities();
  input_port_index_ = this->DeclareInputPort(
      systems::kVectorValued, vec_num).get_index();

  // TODO How to do this for both rgba and depth images?
  this->DeclareAbstractOutputPort();
  // output_port_index_ = DeclareOutputPort(
  //     systems::kVectorValued, get_num_readings()).get_index();
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
  sensors::Image<uint8_t, 4> image(kImageWidth, kImageHeight);
  output->add_port(
      std::make_unique<systems::Value<sensors::Image<uint8_t, 4>>>(image));

  return std::unique_ptr<SystemOutput<double>>(output.release());
}

// TODO(kunimatsu) Add support for sensor frame rate.
void RGBDCamera::DoCalcOutput(const systems::Context<double>& context,
                              systems::SystemOutput<double>* output) const {
  DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(System<double>::CheckValidOutput(output));

  if (context.get_time() == 0.) {
    return;
  }

  const BasicVector<double>* input_vector = this->EvalVectorInput(context,
                                                                  kPortIndex);
  impl_->DoCalcOutput(*input_vector, output);
}

uint32_t RGBDCamera::get_num_readings() const {
  return image_width() * image_height() * 2;
}

uint32_t RGBDCamera::image_width() const {
  return kImageWidth;
}

uint32_t RGBDCamera::image_height() const {
  return kImageHeight;
}

double RGBDCamera::cx() const {
  return image_width() * 0.5;
}

double RGBDCamera::cy() const {
  return image_height() * 0.5;
}

double RGBDCamera::fx() const {
  return image_width() * 0.5 / std::tan(
      0.5 * image_width() / image_height() * kVerticalFovDeg / kRadToDeg);
}

double RGBDCamera::fy() const {
  return image_height() * 0.5 / std::tan(0.5 * kVerticalFovDeg / kRadToDeg);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
