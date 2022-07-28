#include "drake/visualization/colorize_depth_image.h"

#include <algorithm>

namespace drake {
namespace visualization {

using systems::Context;
using systems::LeafSystem;
using systems::SystemTypeTag;
using systems::sensors::Image;
using systems::sensors::ImageDepth16U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageRgba8U;
using systems::sensors::ImageTraits;
using systems::sensors::PixelType;

template <typename T>
ColorizeDepthImage<T>::ColorizeDepthImage()
    : ColorizeDepthImage(PixelType::kDepth32F) {}

template <typename T>
ColorizeDepthImage<T>::ColorizeDepthImage(PixelType depth_pixel_type)
    : LeafSystem<T>(SystemTypeTag<ColorizeDepthImage>{}),
      depth_pixel_type_{depth_pixel_type} {
  switch (depth_pixel_type_) {
    case PixelType::kDepth32F: {
      this->DeclareAbstractInputPort(
          "depth_image_32f", Value<ImageDepth32F>{1, 1});
      break;
    }
    case PixelType::kDepth16U: {
      this->DeclareAbstractInputPort(
          "depth_image_16u", Value<ImageDepth16U>{1, 1});
      break;
    }
    case PixelType::kRgb8U:
    case PixelType::kBgr8U:
    case PixelType::kRgba8U:
    case PixelType::kBgra8U:
    case PixelType::kGrey8U:
    case PixelType::kLabel16I:
    case PixelType::kExpr:
      break;
  }
  this->DeclareAbstractOutputPort("color_image", ImageRgba8U{1, 1},
      &ColorizeDepthImage<T>::CalcOutput);
}

template <typename T>
template <typename U>
ColorizeDepthImage<T>::ColorizeDepthImage(const ColorizeDepthImage<U>& other)
    : ColorizeDepthImage<T>(other.depth_pixel_type_) {}

template <typename T>
ColorizeDepthImage<T>::~ColorizeDepthImage() = default;

namespace {

template <PixelType pixel_type>
void CalcImpl(const Image<pixel_type>& input, ImageRgba8U* output) {
  using T = typename Image<pixel_type>::T;  // Either uint16 or float.

  // Match the output size to the input size.
  if ((input.width() != output->width()) ||
      (input.height() != output->height())) {
    output->resize(input.width(), input.height());
  }

  // We'll use a dynamic range when colorizing, so we need to find the extreme
  // range values first, while ignoring the non-physical values. As we do that
  // scan, it's a convenient time to colorize the non-physical values.
  bool empty = true;
  T minimal = ImageTraits<pixel_type>::kTooFar;
  T maximal = ImageTraits<pixel_type>::kTooClose;
  for (int x = 0; x < input.width(); ++x) {
    for (int y = 0; y < input.height(); ++y) {
      const T value = input.at(x, y)[0];
      if (value <= ImageTraits<pixel_type>::kTooClose) {
        // Use a bright yellow.
        output->at(x, y)[0] = 255;
        output->at(x, y)[1] = 255;
        output->at(x, y)[2] = 0;
        output->at(x, y)[3] = 255;
        continue;
      }
      if (value >= ImageTraits<pixel_type>::kTooFar) {
        // Use a dim red.
        output->at(x, y)[0] = 100;
        output->at(x, y)[1] = 0;
        output->at(x, y)[2] = 0;
        output->at(x, y)[3] = 255;
        continue;
      }
      minimal = std::min(minimal, value);
      maximal = std::max(maximal, value);
      empty = false;
    }
  }

  // If the depth image had no physical returns, then we're done.
  if (empty) {
    return;
  }

  // We'll scale using the minimal and maximal values, where white is nearest.
  //  whiteness = (MAX - depth) / (MAX - MIN)
  const float scale = 1.0f / (maximal - minimal);
  for (int x = 0; x < input.width(); ++x) {
    for (int y = 0; y < input.height(); ++y) {
      const T value = input.at(x, y)[0];
      if (value <= ImageTraits<pixel_type>::kTooClose) {
        // Already set in the min/max loop above.
        continue;
      }
      if (value >= ImageTraits<pixel_type>::kTooFar) {
        // Already set in the min/max loop above.
        continue;
      }
      const float whiteness = (maximal - value) * scale;
      output->at(x, y)[0] = whiteness * 255;
      output->at(x, y)[1] = whiteness * 255;
      output->at(x, y)[2] = whiteness * 255;
      output->at(x, y)[3] = 255;
    }
  }
}

}  // namespace

template <typename T>
void ColorizeDepthImage<T>::Calc(
    const ImageDepth32F& input, ImageRgba8U* output) {
  DRAKE_THROW_UNLESS(output != nullptr);
  CalcImpl(input, output);
}

template <typename T>
void ColorizeDepthImage<T>::Calc(
    const ImageDepth16U& input, ImageRgba8U* output) {
  DRAKE_THROW_UNLESS(output != nullptr);
  CalcImpl(input, output);
}

template <typename T>
void ColorizeDepthImage<T>::CalcOutput(const Context<T>& context,
                                       ImageRgba8U* color_image) const {
  switch (depth_pixel_type_) {
    case PixelType::kDepth32F: {
      const ImageDepth32F& depth_image =
          this->get_input_port().template Eval<ImageDepth32F>(context);
      Calc(depth_image, color_image);
      return;
    }
    case PixelType::kDepth16U: {
      const ImageDepth16U& depth_image =
          this->get_input_port().template Eval<ImageDepth16U>(context);
      Calc(depth_image, color_image);
      return;
    }
    case PixelType::kRgb8U:
    case PixelType::kBgr8U:
    case PixelType::kRgba8U:
    case PixelType::kBgra8U:
    case PixelType::kGrey8U:
    case PixelType::kLabel16I:
    case PixelType::kExpr:
      break;
  }
  DRAKE_UNREACHABLE();
}

}  // namespace visualization
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::visualization::ColorizeDepthImage)
