#include "drake/geometry/common/polygon_surface_mesh_field.h"

namespace drake {
namespace geometry {

template class MeshFieldLinear<double, PolygonSurfaceMesh<double>>;
template class MeshFieldLinear<AutoDiffXd, PolygonSurfaceMesh<AutoDiffXd>>;

}  // namespace geometry
}  // namespace drake
