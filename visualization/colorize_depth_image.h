#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace visualization {

/** Converts a depth image to an RGB image for easier visualization.

The depth values are displayed as grayscale, with black being the furthest
depth, and white being the closest depth. The special values "too near" and
"too far" are colored yellow and red, respectively.

The scaling is dynamic based on the input image depths, not any physical unit.
That means that a particular grayscale value will denote different depths from
one image to the next, based on the dynamic scale of the input image.

@system
name: ColorizeDepthImage
input_ports:
- depth_image_32f (or depth_image_16u)
output_ports:
- color_image
@endsystem

@tparam_default_scalar
@ingroup visualization */
template <typename T>
class ColorizeDepthImage final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ColorizeDepthImage)

  /** Constructs a converter that expects ImageDepth32F input. */
  ColorizeDepthImage();

  /** Constructs a converter that expects inputs of the given depth_pixel_type.
  @param depth_pixel_type must be either kDepth32F or kDepth16U. */
  explicit ColorizeDepthImage(systems::sensors::PixelType depth_pixel_type);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit ColorizeDepthImage(const ColorizeDepthImage<U>&);

  ~ColorizeDepthImage() final;

  /** Convert a depth image to a color image directly (not as a System). */
  static void Calc(const systems::sensors::ImageDepth32F& input,
                   systems::sensors::ImageRgba8U* output);

  /** Convert a depth image to a color image directly (not as a System). */
  static void Calc(const systems::sensors::ImageDepth16U& input,
                   systems::sensors::ImageRgba8U* output);

 private:
  template <typename U> friend class ColorizeDepthImage;

  const systems::sensors::PixelType depth_pixel_type_;

  void CalcOutput(const systems::Context<T>& context,
                  systems::sensors::ImageRgba8U* color_image) const;
};

}  // namespace visualization
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::ColorizeDepthImage)
