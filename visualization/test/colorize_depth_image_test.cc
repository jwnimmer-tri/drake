#include "drake/visualization/colorize_depth_image.h"

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace sensors {
// TODO(jwnimmer-tri) Hoist this to a common location.
template <PixelType pixel_type>
void PrintTo(const Image<pixel_type>& image, std::ostream* out) {
  constexpr int num_channels = Image<pixel_type>::kNumChannels;
  for (int i = 0; i < num_channels; ++i) {
    Eigen::MatrixXd data(image.height(), image.width());
    for (int r = 0; r < image.height(); ++r) {
      for (int c = 0; c < image.width(); ++c) {
        data(r, c) = image.at(c, r)[i];
      }
    }
    *out << "\nchannel " << i << ":\n" << data;
  }
}
}  // namespace sensors
}  // namespace systems
namespace visualization {
namespace {

using systems::sensors::PixelType;
using systems::sensors::ImageDepth16U;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageRgba8U;

class ColorizeDepthTest : public ::testing::Test {
 protected:
  // The depth input is like so:
  //   1.000 1.125 1.250 1.375
  //   1.500 1.625 1.750 1.875
  // The upper left is the *nearest* depth, so will be white.
  // The lower right is the *furthest* deph, so will be black.
  ImageDepth32F get_smooth_input_32() {
    ImageDepth32F input(4, 2);
    for (int x = 0; x < 4; ++x) {
      for (int y = 0; y < 2; ++y) {
        const int i = y * 4 + x;
        input.at(x, y)[0] = 1.0 + (i / 8.0);
      }
    }
    return input;
  }

  ImageDepth16U get_smooth_input_16() {
    ImageDepth16U input(4, 2);
    for (int x = 0; x < 4; ++x) {
      for (int y = 0; y < 2; ++y) {
        const int i = y * 4 + x;
        input.at(x, y)[0] = 100 + (128 * i / 8);
      }
    }
    return input;
  }

  // Smooth input should yield evenly-distributed grayscale pixels.
  ImageRgba8U get_smooth_expected() {
    ImageRgba8U expected(4, 2, 0);
    for (int x = 0; x < 4; ++x) {
      for (int y = 0; y < 2; ++y) {
        const int i = y * 4 + x;
        const int grey = (7 - i) * 255 / 7;
        const int alpha = 255;
        expected.at(x, y)[0] = grey;
        expected.at(x, y)[1] = grey;
        expected.at(x, y)[2] = grey;
        expected.at(x, y)[3] = alpha;
      }
    }
    return expected;
  }
};

TEST_F(ColorizeDepthTest, SmoothCalc32) {
  ImageRgba8U output;
  ColorizeDepthImage<double>::Calc(get_smooth_input_32(), &output);
  EXPECT_EQ(output, get_smooth_expected());
}

TEST_F(ColorizeDepthTest, SmoothCalc16) {
  ImageRgba8U output;
  ColorizeDepthImage<double>::Calc(get_smooth_input_16(), &output);
  EXPECT_EQ(output, get_smooth_expected());
}

TEST_F(ColorizeDepthTest, SmoothEval32) {
  const ColorizeDepthImage<double> dut;
  auto context = dut.CreateDefaultContext();
  dut.get_input_port().FixValue(context.get(), get_smooth_input_32());
  const auto& output = dut.get_output_port().Eval<ImageRgba8U>(*context);
  EXPECT_EQ(output, get_smooth_expected());
}

TEST_F(ColorizeDepthTest, SmoothEval16) {
  const ColorizeDepthImage<double> dut(PixelType::kDepth16U);
  auto context = dut.CreateDefaultContext();
  dut.get_input_port().FixValue(context.get(), get_smooth_input_16());
  const auto& output = dut.get_output_port().Eval<ImageRgba8U>(*context);
  EXPECT_EQ(output, get_smooth_expected());
}

TEST_F(ColorizeDepthTest, ScalarConversion) {
  const ColorizeDepthImage<double> dut;
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [](const auto& converted) {
    EXPECT_EQ(converted.get_input_port().get_name(), "depth_image_32f");
  }));
  EXPECT_TRUE(is_symbolic_convertible(dut, [](const auto& converted) {
    EXPECT_EQ(converted.get_input_port().get_name(), "depth_image_32f");
  }));

  const ColorizeDepthImage<double> dut2(PixelType::kDepth16U);
  EXPECT_TRUE(is_autodiffxd_convertible(dut2, [](const auto& converted) {
    EXPECT_EQ(converted.get_input_port().get_name(), "depth_image_16u");
  }));
  EXPECT_TRUE(is_symbolic_convertible(dut2, [](const auto& converted) {
    EXPECT_EQ(converted.get_input_port().get_name(), "depth_image_16u");
  }));
}

}  // namespace
}  // namespace visualization
}  // namespace drake
