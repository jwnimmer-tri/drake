#pragma once

#include <Eigen/Dense>

#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace sensors {

/// RGB-D camera model to provide both RGB and depth images at the fixed frame
/// rate (30Hz) and image resolution (VGA).  The RGB-D camera's base frame is
/// defined to be x-right, y-forward, z-up and the RGB camera optical frame is
/// -20mm offset w.r.t the base frame's x axis.  The depth camera optical frame
/// is at the same position as rgb camera optical frame, so that users can treat
/// the depth image as "registered" to the rgb image.  No disparity is
/// considered for now.  The origin of rendered image is at the corner of left-
/// bottom and the direction of axes are x-right, y-up.
/// The depth range for rendering is from 0.5m to 5.0m and is shared between rgb
/// and depth images for now.
// TODO(kunimatsu.hashimoto) Change the camera base frame's orientation to be
// x-forward, y-left, z-up.
class RGBDCamera : public LeafSystem<double> {
 public:
  /// A constructor for %RGBDCamera
  ///
  /// @param[in] name The name of the rgbd camera. This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param[in] tree The RigidBodyTree containing the geometric configuration
  /// of the world. This parameter is aliased by a class member variable. Thus,
  /// its life span must exceed that of this class's instance.
  ///
  /// @param[in] position 3D position for RGBDCamera
  ///
  /// @param[in] orientation 3D orientation (roll, pitch, yaw) for RGBDCamera
  ///
  /// @param[in] show_window The flag to show visible window. If this is false,
  /// offscreen rendering is executed.
  RGBDCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const Eigen::Vector3d& position,
             const Eigen::Vector3d& orientation,
             bool show_window);

  /// A constructor for %RGBDCamera
  ///
  /// @param[in] name The name of the rgbd camera. This can be any value, but
  /// should typically be unique among all sensors attached to a particular
  /// model instance.
  ///
  /// @param[in] tree The RigidBodyTree containing the geometric configuration
  /// of the world. This parameter is aliased by a class member variable. Thus,
  /// its life span must exceed that of this class's instance.
  ///
  /// @param[in] frame The frame to which this camera is attached.
  ///
  /// @param[in] show_window The flag to show visible window. If this is false,
  /// offscreen rendering is executed.
  RGBDCamera(const std::string& name,
             const RigidBodyTree<double>& tree,
             const RigidBodyFrame<double>& frame,
             bool show_window);

  ~RGBDCamera();
  // Non-copyable.
  /// @name Deleted Copy/Move Operations
  /// DepthSensor is neither copyable nor moveable.
  ///@{
  explicit RGBDCamera(const RGBDCamera&) = delete;
  RGBDCamera& operator=(const RGBDCamera&) = delete;
  ///@}
  /*
  /// Returns the name of this sensor. The name can be any user-specified value.
  // const std::string& get_name() const;  // TODO is this needed? overriden?

  /// Returns the RigidBodyTree that this sensor is sensing.
  const RigidBodyTree<double>& get_tree() const;

  /// Returns this sensor's frame, which specifies its location and orientation
  /// in the RigidBodyTree.
  const RigidBodyFrame<double>& get_frame() const;

  /// Returns a descriptor of the input port containing the generalized state of
  /// the RigidBodyTree.
  const InputPortDescriptor<double>& get_rigid_body_tree_state_input_port()
      const;

  /// Returns a descriptor of the state output port, which contains the sensor's
  /// sensed values.
  const OutputPortDescriptor<double>& get_sensor_state_output_port() const;
  */
  /// Allocates the output vector. See this class' description for details of
  /// this output vector.
  std::unique_ptr<SystemOutput<double>> AllocateOutput(
    const Context<double>& context) const override;

  uint32_t get_num_readings() const;
  /// The image width in pixels.
  uint32_t image_width() const;
  /// The image height in pixels.
  uint32_t image_height() const;
  /// The center of image for x direction in pixels.
  double cx() const;
  /// The center of image for y direction in pixels.
  double cy() const;
  /// The focal length for x direction in pixels.
  double fx() const;
  /// The focal length for y direction in pixels.
  double fy() const;

  // void EvalOutput(const systems::Context<double>& context,
  //                 systems::SystemOutput<double>* output) const override;
 protected:
  /// Update all the model frames for renderer and outputs the rendered images.
  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;

  int input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
