#pragma once

#include <Eigen/Dense>

#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace systems {
namespace sensors {
/// RGB-D camera model to provide both RGB and depth images. The image
/// resolution is fixed at VGA (640x480) for now.  The RGB-D camera's base frame
/// is defined to be x-right, y-forward, z-up and the origin of the rgb camera
/// optical frame is -20mm offset w.r.t the base frame's x axis.  The origin of
/// depth camera optical frame is at the same position as the origin of the rgb
/// camera optical frame, and so users can treat the depth images like as
/// "registered" to the rgb image.  No disparity is considered for now.
/// The origin of rendered image is at the upper-left the direction of axes are
/// x-right, y-down, and z-forward.
/// The depth range for rendering is from 0.5m to 5.0m and is shared between rgb
/// and depth images for now.
// TODO(kunimatsu-tri) Change the camera base frame's orientation to be
// x-forward, y-left, z-up.
class RGBDCamera : public LeafSystem<double> {
 public:
  enum CameraType {
    kColor = 0,
    kDepth,
  };
  /// A constructor for %RGBDCamera
  ///
  /// @param name The name of the rgbd camera. This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric configuration
  /// of the world. This parameter is aliased by a class member variable. Thus,
  /// its life span must exceed that of this class's instance.
  ///
  /// @param position 3D position for RGBDCamera
  /// @param orientation 3D orientation (roll, pitch, yaw) for RGBDCamera
  /// @param frame_rate Updates output at this frame rate in Hz.
  /// @param show_window To show visible window.  If this is false, offscreen
  /// rendering is executed.
  RGBDCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation,
             double frame_rate,
             double fov_y,
             bool show_window);

  /// A constructor for %RGBDCamera
  ///
  /// @param name The name of the rgbd camera. This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param tree The RigidBodyTree containing the geometric configuration
  /// of the world. This parameter is aliased by a class member variable. Thus,
  /// its life span must exceed that of this class's instance.
  ///
  /// @param frame The frame to which this camera is attached.
  ///
  /// @param show_window The flag to show visible window. If this is false,
  /// offscreen rendering is executed.
  RGBDCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const RigidBodyFrame<double>& frame,
             double frame_rate,
             double fov_y,
             bool show_window);

  ~RGBDCamera();
  // Non-copyable.
  /// @name Deleted Copy/Move Operations
  /// RGBDCamera is neither copyable nor moveable.
  ///@{
  explicit RGBDCamera(const RGBDCamera&) = delete;
  RGBDCamera& operator=(const RGBDCamera&) = delete;
  explicit RGBDCamera(RGBDCamera&&) = delete;
  RGBDCamera& operator=(RGBDCamera&&) = delete;
  ///@}

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const;

  /// Allocates the output vector. See this class' description for details of
  /// this output vector.
  std::unique_ptr<SystemOutput<double>> AllocateOutput(
    const Context<double>& context) const override;

  // TODO(kunimatsu-tri) Write comment
  const CameraInfo& get_camera_info(const CameraType type) const;

  // TODO(kunimatsu-tri) Add API to provide transformation from base to sensor

 protected:
  /// Update all the model frames for renderer and outputs the rendered images.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;

  int input_port_index_{};
  const double frame_interval_{};
  // For the time step calculation in const member function
  mutable double previous_output_time_{0.};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
