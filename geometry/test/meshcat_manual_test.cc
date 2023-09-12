#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/find_runfiles.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/sensors/image_writer.h"

DEFINE_int32(image_width, 100, "");
DEFINE_int32(image_height, 100, "");
DEFINE_double(image_timeout, 1.0, "");

/* To test, you must manually run `bazel run //geometry:meshcat_manual_test`,
then follow the instructions on your console. */

namespace drake {
namespace geometry {

using Eigen::Vector3d;
using common::MaybePauseForUser;
using math::RigidTransformd;
using math::RotationMatrixd;

int do_main() {
  auto meshcat = std::make_shared<Meshcat>();

  // For every two items we add to the initial array, decrement start_x by one
  // to keep things centered.
  // Use ++x as the x-position of new items.
  const double start_x = -8;
  double x = start_x;

  Vector3d sphere_home{++x, 0, 0};
  meshcat->SetObject("sphere", Sphere(0.25), Rgba(1.0, 0, 0, 1));
  meshcat->SetTransform("sphere", RigidTransformd(sphere_home));

  meshcat->SetObject("cylinder", Cylinder(0.25, 0.5), Rgba(0.0, 1.0, 0, 1));
  meshcat->SetTransform("cylinder", RigidTransformd(Vector3d{++x, 0, 0}));

  // For animation, we'll aim the camera between the cylinder and ellipsoid.
  const Vector3d animation_target{x + 0.5, 0, 0};

  meshcat->SetObject("ellipsoid", Ellipsoid(0.25, 0.25, 0.5),
                     Rgba(1.0, 0, 1, 0.5));
  meshcat->SetTransform("ellipsoid", RigidTransformd(Vector3d{++x, 0, 0}));

  Vector3d box_home{++x, 0, 0};
  meshcat->SetObject("box", Box(0.25, 0.25, 0.5), Rgba(0, 0, 1, 1));
  meshcat->SetTransform("box", RigidTransformd(box_home));

  meshcat->SetObject("capsule", Capsule(0.25, 0.5), Rgba(0, 1, 1, 1));
  meshcat->SetTransform("capsule", RigidTransformd(Vector3d{++x, 0, 0}));

  // Note that height (in z) is the first argument.
  meshcat->SetObject("cone", MeshcatCone(0.5, 0.25, 0.5), Rgba(1, 0, 0, 1));
  meshcat->SetTransform("cone", RigidTransformd(Vector3d{++x, 0, 0}));

  // The color and shininess properties come from PBR materials.
  meshcat->SetObject(
      "gltf",
      Mesh(FindResourceOrThrow("drake/geometry/render/test/meshes/cube.gltf"),
           0.25));
  const Vector3d gltf_pose{++x, 0, 0};
  meshcat->SetTransform("gltf", RigidTransformd(gltf_pose));

  auto mustard_obj =
      FindRunfile("drake_models/ycb/meshes/006_mustard_bottle_textured.obj")
          .abspath;
  meshcat->SetObject("mustard", Mesh(mustard_obj, 3.0));
  meshcat->SetTransform("mustard", RigidTransformd(Vector3d{++x, 0, 0}));

  {
    const int kPoints = 100000;
    perception::PointCloud cloud(
        kPoints, perception::pc_flags::kXYZs | perception::pc_flags::kRGBs);
    Eigen::Matrix3Xf m = Eigen::Matrix3Xf::Random(3, kPoints);
    cloud.mutable_xyzs() = Eigen::DiagonalMatrix<float, 3>{0.25, 0.25, 0.5} * m;
    cloud.mutable_rgbs() = (255.0 * (m.array() + 1.0) / 2.0).cast<uint8_t>();
    meshcat->SetObject("point_cloud", cloud, 0.01);
    meshcat->SetTransform("point_cloud", RigidTransformd(Vector3d{++x, 0, 0}));
  }

  {
    Eigen::Matrix3Xd vertices(3, 200);
    Eigen::RowVectorXd t = Eigen::RowVectorXd::LinSpaced(200, 0, 10 * M_PI);
    vertices << 0.25 * t.array().sin(), 0.25 * t.array().cos(), t / (10 * M_PI);
    meshcat->SetLine("line", vertices, 3.0, Rgba(0, 0, 1, 1));
    meshcat->SetTransform("line", RigidTransformd(Vector3d{++x, 0, -0.5}));
  }

  {
    Eigen::Matrix3Xd start(3, 4), end(3, 4);
    // clang-format off
    start << -0.1, -0.1,  0.1,  0.1,
             -0.1,  0.1, -0.1,  0.1,
                0,    0,    0,    0;
    // clang-format on
    end = start;
    end.row(2) = Eigen::RowVector4d::Ones();
    meshcat->SetLineSegments("line_segments", start, end, 5.0,
                             Rgba(0, 1, 0, 1));
    meshcat->SetTransform("line_segments",
                          RigidTransformd(Vector3d{++x, 0, -0.5}));
  }

  // The TriangleSurfaceMesh variant of SetObject calls SetTriangleMesh(), so
  // visually inspecting the results of TriangleSurfaceMesh is sufficient here.
  {
    const int face_data[2][3] = {{0, 1, 2}, {2, 3, 0}};
    std::vector<SurfaceTriangle> faces;
    for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
    const Vector3d vertex_data[4] = {
        {0, 0, 0}, {0.5, 0, 0}, {0.5, 0.5, 0}, {0, 0.5, 0.5}};
    std::vector<Vector3d> vertices;
    for (int v = 0; v < 4; ++v) vertices.emplace_back(vertex_data[v]);
    TriangleSurfaceMesh<double> surface_mesh(
        std::move(faces), std::move(vertices));
    meshcat->SetObject("triangle_mesh", surface_mesh, Rgba(0.9, 0, 0.9, 1.0));
    meshcat->SetTransform("triangle_mesh",
                          RigidTransformd(Vector3d{++x, -0.25, 0}));

    meshcat->SetObject("triangle_mesh_wireframe", surface_mesh,
                       Rgba(0.9, 0, 0.9, 1.0), true, 5.0);
    meshcat->SetTransform("triangle_mesh_wireframe",
                          RigidTransformd(Vector3d{++x, -0.25, 0}));
  }

  // SetTriangleColorMesh.
  {
    // clang-format off
    Eigen::Matrix3Xd vertices(3, 4);
    vertices <<
      0, 0.5, 0.5, 0,
      0, 0,   0.5, 0.5,
      0, 0,   0,   0.5;
    Eigen::Matrix3Xi faces(3, 2);
    faces <<
      0, 2,
      1, 3,
      2, 0;
    Eigen::Matrix3Xd colors(3, 4);
    colors <<
      1, 0, 0, 1,
      0, 1, 0, 1,
      0, 0, 1, 0;
    // clang-format on
    meshcat->SetTriangleColorMesh("triangle_color_mesh", vertices, faces,
                                  colors);
    meshcat->SetTransform("triangle_color_mesh",
                          RigidTransformd(Vector3d{++x, -0.25, 0}));
  }

  // PlotSurface.
  {
    constexpr int nx = 15, ny = 11;
    Eigen::MatrixXd X =
        Eigen::RowVectorXd::LinSpaced(nx, 0, 1).replicate<ny, 1>();
    Eigen::MatrixXd Y = Eigen::VectorXd::LinSpaced(ny, 0, 1).replicate<1, nx>();
    // z = y*sin(5*x)
    Eigen::MatrixXd Z = (Y.array() * (5 * X.array()).sin()).matrix();

    meshcat->PlotSurface("plot_surface", X, Y, Z, Rgba(0, 0, 0.9, 1.0), true);
    meshcat->SetTransform("plot_surface",
                          RigidTransformd(Vector3d{++x, -0.25, 0}));
  }
  drake::log()->set_level(spdlog::level::trace);

  std::cout << R"""(
Open up your browser to the URL above.

- The background should be grey.
- From left to right along the x axis, you should see:
  - a red sphere
  - a green cylinder (with the long axis in z)
  - a pink semi-transparent ellipsoid (long axis in z)
  - a blue box (long axis in z)
  - a teal capsule (long axis in z)
  - a red cone (expanding in +z, twice as wide in y than in x)
  - a shiny, green, dented cube (created with a PBR material)
  - a yellow mustard bottle w/ label
  - a dense rainbow point cloud in a box (long axis in z)
  - a blue line coiling up (in z).
  - 4 green vertical line segments (in z).
  - a purple triangle mesh with 2 faces.
  - the same purple triangle mesh drawn as a wireframe.
  - the same triangle mesh drawn in multicolor.
  - a blue mesh plot of the function z = y*sin(5*x).
)""";
  MaybePauseForUser();

  std::cout << "Capturing..." << std::endl;
  auto image = meshcat->CaptureImage(FLAGS_image_width, FLAGS_image_height, FLAGS_image_timeout);
  std::cout << "Finished; saving..." << std::endl;
  SaveToPng(image, "/home/jwnimmer/Downloads/image.png");
  std::cout << "Saved." << std::endl;

  std::cout << "Calling meshcat.Flush(), which will block until all clients "
               "have received all the data)...";
  meshcat->Flush();
  std::cout << "Exiting..." << std::endl;
  return 0;
}

}  // namespace geometry
}  // namespace drake

int main(int argc, char** argv) {
  google::SetUsageMessage(" ");  // Nerf a silly warning emitted by gflags.
  google::ParseCommandLineFlags(&argc, &argv, true);
  return drake::geometry::do_main();
}
