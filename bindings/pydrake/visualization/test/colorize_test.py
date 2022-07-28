import pydrake.visualization as mut

import unittest

from pydrake.systems.sensors import (
    ImageDepth16U,
    ImageDepth32F,
    ImageRgba8U,
    PixelType,
)


class TestConfig(unittest.TestCase):

    def test_colorize_depth_image_ctor(self):
        """Exercises ColorizeDepthImage constructors."""
        mut.ColorizeDepthImage()
        mut.ColorizeDepthImage(depth_pixel_type=PixelType.kDepth16U)

    def test_colorize_depth_image_calc_32(self):
        """Exercises ColorizeDepthImage::Calc."""
        depth = ImageDepth32F(20, 10)
        result = ImageRgba8U(1, 1)
        mut.ColorizeDepthImage.Calc(depth, result)
        self.assertEqual(result.width(), 20)
        self.assertEqual(result.height(), 10)

    def test_colorize_depth_image_calc_16(self):
        """Exercises ColorizeDepthImage::Calc."""
        depth = ImageDepth16U(20, 10)
        result = ImageRgba8U(1, 1)
        mut.ColorizeDepthImage.Calc(depth, result)
        self.assertEqual(result.width(), 20)
        self.assertEqual(result.height(), 10)
