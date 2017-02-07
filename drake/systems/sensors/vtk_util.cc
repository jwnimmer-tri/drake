#include "drake/systems/sensors/vtk_util.h"

#include <vtkCellData.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

namespace drake {
namespace systems {
namespace sensors {

vtkSmartPointer<vtkPlaneSource> VtkUtil::CreateSquarePlane(
    double size, const unsigned char color[3]) {
  vtkSmartPointer<vtkPlaneSource> plane =
      vtkSmartPointer<vtkPlaneSource>::New();
  const double half_size = size * 0.5;
  plane->SetOrigin(-half_size, -half_size, 0.);
  plane->SetPoint1(-half_size, half_size, 0.);
  plane->SetPoint2(half_size, -half_size, 0.);
  plane->Update();

  vtkSmartPointer<vtkUnsignedCharArray> colors =
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->InsertNextTupleValue(color);
  plane->GetOutput()->GetCellData()->SetScalars(colors);

  return plane;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
