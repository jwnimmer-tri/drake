#pragma once

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace drake {
namespace systems {
namespace sensors {
// TODO(kunimatsu.hashimoto) Write Doxygen comments.
// TODO(kunimatsu.hashimoto) Replace this with FlatTerrainSource
vtkSmartPointer<vtkPolyData> CreateFlatTerrain(
    double size, const unsigned char color[3]);

// class vtkFratTerrainSource {
//  public:
//   vtkFratTerrainSource();

//   void SetXLength(double x) { x_ = x; }
//   void SetYLength(double y) { y_ = y; }
//   hogehoge GetOutputPort();

//  private:
//   double x_;
//   double y_;
// };

}  // namespace sensors
}  // namespace systems
}  // namespace drake
