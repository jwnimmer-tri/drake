#include "drake/systems/sensors/image_io_png.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

#include <png.h>

#include "drake/common/drake_assert.h"
#include "drake/common/scope_exit.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

// A position within a byte stream.
struct BoundedCursor {
  png_const_bytep position{nullptr};
  png_size_t remaining{0};
};

// Read `size` bytes from the png file into `data`.
void PngReadCallback(png_structp png_ptr, png_bytep data, png_size_t size) {
  DRAKE_DEMAND(png_ptr != nullptr);
  void* io_ptr = png_get_io_ptr(png_ptr);
  DRAKE_DEMAND(io_ptr != nullptr);
  BoundedCursor* cursor = reinterpret_cast<BoundedCursor*>(io_ptr);      
  if (size > cursor->remaining) {
    png_error(png_ptr, "Attempt to read past end of PNG input buffer");
  }
  std::memcpy(data, cursor->position, size);
  cursor->position += size;
  cursor->remaining -= size;
}

template <PixelType kPixelType>
void DoLoadPngData(
    const void* data, size_t data_size, Image<kPixelType>* image,
    const DiagnosticPolicy& diagnostic) {
  using ChannelType = typename ImageTraits<kPixelType>::ChannelType;
  constexpr int bit_depth = 8 * sizeof(ChannelType);
  constexpr int num_channels = ImageTraits<kPixelType>::kNumChannels;

  // These are variables used by PngReadCallback whiel reading `data`.
  BoundedCursor cursor{
      .position = static_cast<png_const_bytep>(data),
      .remaining = data_size};

  // We need to stack-allocate this prior to calling setjmp, because it is not
  // trivially destructable.
  std::vector<png_bytep> row_pointers;

  png_structp png_ptr = png_create_read_struct(
      PNG_LIBPNG_VER_STRING, 0, 0, 0);
  DRAKE_DEMAND(png_ptr != nullptr);

  png_infop info_ptr = png_create_info_struct(png_ptr);
  DRAKE_DEMAND(info_ptr != nullptr);
  ScopeExit guard([&png_ptr, &info_ptr]() {
    png_destroy_read_struct(&png_ptr, &info_ptr, 0);
  });

  // Initialize the libpng error handling based on non-local gotos.  Any local
  // variables introduced after this point must be trivially destructible!
  if (setjmp(png_jmpbuf(png_ptr)) != 0) {
    // The non-local goto put us back here.
    diagnostic.Error("Error during PNG decoding");
    return;
  }

  // Read the png header and ensure it matches out image's Pixel type.
  png_set_read_fn(png_ptr, &cursor, PngReadCallback);
  png_read_info(png_ptr, info_ptr);
  if (png_get_bit_depth(png_ptr, info_ptr) != bit_depth) {
    diagnostic.Error("Decoded PNG bit depth mismatch");
    return;
  }
  if (png_get_channels(png_ptr, info_ptr) != num_channels) {
    diagnostic.Error("Decoded PNG num channel mismatch");
    return;
  }
  const png_uint_32 unsigned_width = png_get_image_width(png_ptr, info_ptr);
  const png_uint_32 unsigned_height = png_get_image_height(png_ptr, info_ptr);
  if (unsigned_width > static_cast<size_t>(INT_MAX)) {
    diagnostic.Error("width");
    return;
  }
  if (unsigned_height > static_cast<size_t>(INT_MAX)) {
    diagnostic.Error("height");
    return;
  }
  const int width = static_cast<int>(unsigned_width);
  const int height = static_cast<int>(unsigned_height);

  // Resize the image storage to match what we need.
  if ((width == 0) || (height == 0)) {
    *image = Image<kPixelType>{};
    return;
  }
  if ((image->width() != width) || (image->height() != height)) {
    image->resize(width, height);
  }
  row_pointers.resize(height);
  for (int i = 0; i < height; ++i) {
    row_pointers[i] = reinterpret_cast<png_bytep>(image->at(0, i));
  }

  // Read the whole image.
  if (png_get_color_type(png_ptr, info_ptr) == PNG_COLOR_TYPE_PALETTE) {
    png_set_palette_to_rgb(png_ptr);
  }
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  png_set_swap(png_ptr);
#endif
  png_read_update_info(png_ptr, info_ptr);
  png_read_image(png_ptr, row_pointers.data());
}

template <PixelType kPixelType>
void DoSavePngData(const Image<kPixelType>& image, std::vector<std::byte>* output) {
  (void)(image);
  (void)(output);
}

template <PixelType kPixelType>
void DoSavePngFile(const Image<kPixelType>& image, std::string_view filename) {
  (void)(image);
  (void)(filename);
}

std::vector<char> ReadEntireFile(
    std::string_view filename,
    const DiagnosticPolicy& diagnostic) {
  std::vector<char> result;
  std::ifstream file(std::string(filename), std::ios::binary | std::ios::ate);
  if (!file) {
    diagnostic.Error("no file");
    return result;
  }
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  result.resize(size);
  file.read(result.data(), size);
  if (!file) {
    diagnostic.Error("incomplete read");
  }
  return result;
}

}  // namespace

void LoadPngFile(std::string_view filename, ImageRgb8U* image,
                 const DiagnosticPolicy& diagnostic) {
  auto data = ReadEntireFile(filename, diagnostic);
  LoadPngData(data.data(), data.size(), image, diagnostic);
}

void LoadPngFile(std::string_view filename, ImageBgr8U* image,
                 const DiagnosticPolicy& diagnostic) {
  auto data = ReadEntireFile(filename, diagnostic);
  LoadPngData(data.data(), data.size(), image, diagnostic);
}

void LoadPngFile(std::string_view filename, ImageRgba8U* image,
                 const DiagnosticPolicy& diagnostic) {
  auto data = ReadEntireFile(filename, diagnostic);
  LoadPngData(data.data(), data.size(), image, diagnostic);
}

void LoadPngFile(std::string_view filename, ImageBgra8U* image,
                 const DiagnosticPolicy& diagnostic) {
  auto data = ReadEntireFile(filename, diagnostic);
  LoadPngData(data.data(), data.size(), image, diagnostic);
}

void LoadPngFile(std::string_view filename, ImageGrey8U* image,
                 const DiagnosticPolicy& diagnostic) {
  auto data = ReadEntireFile(filename, diagnostic);
  LoadPngData(data.data(), data.size(), image, diagnostic);
}

void LoadPngFile(std::string_view filename, ImageDepth16U* image,
                 const DiagnosticPolicy& diagnostic) {
  auto data = ReadEntireFile(filename, diagnostic);
  LoadPngData(data.data(), data.size(), image, diagnostic);
}

void LoadPngFile(std::string_view filename, ImageLabel16I* image,
                 const DiagnosticPolicy& diagnostic) {
  auto data = ReadEntireFile(filename, diagnostic);
  LoadPngData(data.data(), data.size(), image, diagnostic);
}

void LoadPngData(const void* data, size_t data_size, ImageRgb8U* image,
                 const DiagnosticPolicy& diagnostic) {
  DoLoadPngData(data, data_size, image, diagnostic);
}

void LoadPngData(const void* data, size_t data_size, ImageBgr8U* image,
                 const DiagnosticPolicy& diagnostic) {
  DoLoadPngData(data, data_size, image, diagnostic);
}

void LoadPngData(const void* data, size_t data_size, ImageRgba8U* image,
                 const DiagnosticPolicy& diagnostic) {
  DoLoadPngData(data, data_size, image, diagnostic);
}

void LoadPngData(const void* data, size_t data_size, ImageBgra8U* image,
                 const DiagnosticPolicy& diagnostic) {
  DoLoadPngData(data, data_size, image, diagnostic);
}

void LoadPngData(const void* data, size_t data_size, ImageGrey8U* image,
                 const DiagnosticPolicy& diagnostic) {
  DoLoadPngData(data, data_size, image, diagnostic);
}

void LoadPngData(const void* data, size_t data_size, ImageDepth16U* image,
                 const DiagnosticPolicy& diagnostic) {
  DoLoadPngData(data, data_size, image, diagnostic);
}

void LoadPngData(const void* data, size_t data_size, ImageLabel16I* image,
                 const DiagnosticPolicy& diagnostic) {
  DoLoadPngData(data, data_size, image, diagnostic);
}

void SavePngFile(const ImageRgb8U& image, std::string_view filename) {
  DoSavePngFile(image, filename);
}

void SavePngFile(const ImageBgr8U& image, std::string_view filename) {
  DoSavePngFile(image, filename);
}

void SavePngFile(const ImageRgba8U& image, std::string_view filename) {
  DoSavePngFile(image, filename);
}

void SavePngFile(const ImageBgra8U& image, std::string_view filename) {
  DoSavePngFile(image, filename);
}

void SavePngFile(const ImageGrey8U& image, std::string_view filename) {
  DoSavePngFile(image, filename);
}

void SavePngFile(const ImageDepth16U& image, std::string_view filename) {
  DoSavePngFile(image, filename);
}

void SavePngFile(const ImageLabel16I& image, std::string_view filename) {
  DoSavePngFile(image, filename);
}

void SavePngData(const ImageRgb8U& image, std::vector<std::byte>* output) {
  DoSavePngData(image, output);
}

void SavePngData(const ImageBgr8U& image, std::vector<std::byte>* output) {
  DoSavePngData(image, output);
}

void SavePngData(const ImageRgba8U& image, std::vector<std::byte>* output) {
  DoSavePngData(image, output);
}

void SavePngData(const ImageBgra8U& image, std::vector<std::byte>* output) {
  DoSavePngData(image, output);
}

void SavePngData(const ImageGrey8U& image, std::vector<std::byte>* output) {
  DoSavePngData(image, output);
}

void SavePngData(const ImageDepth16U& image, std::vector<std::byte>* output) {
  DoSavePngData(image, output);
}

void SavePngData(const ImageLabel16I& image, std::vector<std::byte>* output) {
  DoSavePngData(image, output);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
