#pragma once

#include <string>

#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

/** @name     Utility functions for writing TIFF images to disk.

 Given a fully-specified path to the file to write and corresponding image data,
 these functions will _attempt_ to write the image data to the file. The
 functions assume that the path is valid and writable. These functions will
 attempt to write the image to the given file path. The file format will be
 that indicated by the function name, but the extension will be whatever is
 provided as input.

 These function do not do validation on the provided file path (existence,
 writability, correspondence with image type, etc.) It relies on the caller to
 have done so.  */
//@{

/** Writes the depth (32-bit) image data to disk. Png files do not support
 channels larger than 16-bits and its support for floating point values is
 also limited at best. So, depth images can only be written as tiffs.  */
void SaveToTiff(const ImageDepth32F& image, const std::string& file_path);

//@}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
