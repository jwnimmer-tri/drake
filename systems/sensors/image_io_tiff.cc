#include "drake/systems/sensors/image_io_tiff.h"

#include <cstddef>
#include <cstring>

#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>
#include <vtkTIFFWriter.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

template <PixelType kPixelType>
void SaveToFileHelper(const Image<kPixelType>& image,
                      const std::string& file_path) {
  const int width = image.width();
  const int height = image.height();
  constexpr int num_channels = Image<kPixelType>::kNumChannels;
  using T = typename Image<kPixelType>::T;

  vtkNew<vtkImageData> vtk_image;
  vtk_image->SetDimensions(width, height, 1);
  switch (kPixelType) {
    case PixelType::kDepth32F:
      vtk_image->AllocateScalars(VTK_FLOAT, num_channels);
      break;
    default:
      DRAKE_UNREACHABLE();
  }

  std::byte* cursor = reinterpret_cast<std::byte*>(
      vtk_image->GetScalarPointer());
  for (int v = height - 1; v >= 0; --v) {
    for (int u = 0; u < width; ++u) {
      for (int c = 0; c < num_channels; ++c) {
        const T* element = image.at(u, v) + c;
        std::memcpy(cursor, element, sizeof(T));
        cursor += sizeof(T);
      }
    }
  }

  vtkSmartPointer<vtkImageWriter> writer =
      vtkSmartPointer<vtkTIFFWriter>::New();
  writer->SetFileName(file_path.c_str());
  writer->SetInputData(vtk_image.GetPointer());
  writer->Write();
}

void SaveToTiff(const ImageDepth32F& image, const std::string& file_path) {
  SaveToFileHelper(image, file_path);
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
