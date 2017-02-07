#pragma once

#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>

namespace drake {
namespace systems {
namespace sensors {
// TODO(kunimatsu.hashimoto) Write Doxygen comments.

class VtkUtil {
 public:
  static vtkSmartPointer<vtkPlaneSource> CreateSquarePlane(
    double size, const unsigned char color[3]);
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
