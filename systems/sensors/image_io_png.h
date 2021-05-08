#pragma once

#include <cstddef>
#include <string>
#include <string_view>

#include "drake/systems/sensors/image.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO(jwnimmer-tri) Write me.
struct DiagnosticDetail {
  std::string message;
};

// TODO(jwnimmer-tri) Write me.
struct DiagnosticPolicy {
  std::function<void(const DiagnosticDetail&)> on_warning;
  std::function<void(const DiagnosticDetail&)> on_error;

  void Warning(std::string) const;
  void Error(std::string) const;
  void Warning(const DiagnosticDetail&) const;
  void Error(const DiagnosticDetail&) const;
};

/** @name Functions for reading and writing PNG images.

Only the following pixel types are supported:
- kRgb8U
- kBgr8U
- kRgba8U
- kBgra8U
- kGrey8U
- kDepth16U
- kLabel16I

The following pixel types are NOT supported:
- kDepth32F
- kExpr

 TODO(jwnimmer-tri) Rewrite me.
 Given a fully-specified path to the file to write and corresponding image data,
 these functions will _to write the image data to the file. The
 functions assume that the path is valid and writable. These functions will
 attempt to write the image to the given file path. The file format will be
 that indicated by the function name, but the extension will be whatever is
 provided as input.

*/
//@{

void LoadPngFile(std::string_view filename, ImageRgb8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngFile(std::string_view filename, ImageBgr8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngFile(std::string_view filename, ImageRgba8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngFile(std::string_view filename, ImageBgra8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngFile(std::string_view filename, ImageGrey8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngFile(std::string_view filename, ImageDepth16U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngFile(std::string_view filename, ImageLabel16I* image,
                 const DiagnosticPolicy& diagnostic = {});

void LoadPngData(const void* data, size_t data_size, ImageRgb8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngData(const void* data, size_t data_size, ImageBgr8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngData(const void* data, size_t data_size, ImageRgba8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngData(const void* data, size_t data_size, ImageBgra8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngData(const void* data, size_t data_size, ImageGrey8U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngData(const void* data, size_t data_size, ImageDepth16U* image,
                 const DiagnosticPolicy& diagnostic = {});
void LoadPngData(const void* data, size_t data_size, ImageLabel16I* image,
                 const DiagnosticPolicy& diagnostic = {});

void SavePngFile(const ImageRgb8U& image, std::string_view filename);
void SavePngFile(const ImageBgr8U& image, std::string_view filename);
void SavePngFile(const ImageRgba8U& image, std::string_view filename);
void SavePngFile(const ImageBgra8U& image, std::string_view filename);
void SavePngFile(const ImageGrey8U& image, std::string_view filename);
void SavePngFile(const ImageDepth16U& image, std::string_view filename);
void SavePngFile(const ImageLabel16I& image, std::string_view filename);

void SavePngData(const ImageRgb8U& image, std::vector<std::byte>* output);
void SavePngData(const ImageBgr8U& image, std::vector<std::byte>* output);
void SavePngData(const ImageRgba8U& image, std::vector<std::byte>* output);
void SavePngData(const ImageBgra8U& image, std::vector<std::byte>* output);
void SavePngData(const ImageGrey8U& image, std::vector<std::byte>* output);
void SavePngData(const ImageDepth16U& image, std::vector<std::byte>* output);
void SavePngData(const ImageLabel16I& image, std::vector<std::byte>* output);

//@}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
