#include "drake/geometry/meshcat_internal.h"

#include <algorithm>
#include <regex>
#include <stdexcept>
#include <utility>

#include <fmt/format.h>
#include <uuid.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/overloaded.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace internal {

namespace fs = std::filesystem;

using drake::internal::DiagnosticPolicy;
using math::RigidTransformd;
using math::RotationMatrixd;

namespace {

std::string LoadResource(const std::string& resource_name) {
  const std::string resource = FindResourceOrThrow(resource_name);
  std::optional<std::string> content = ReadFile(resource);
  if (!content) {
    throw std::runtime_error(
        fmt::format("Error opening resource: {}", resource_name));
  }
  return std::move(*content);
}

}  //  namespace

std::optional<std::string_view> GetMeshcatStaticResource(
    std::string_view url_path) {
  static const drake::never_destroyed<std::string> meshcat_js(
      LoadResource("drake/geometry/meshcat.js"));
  static const drake::never_destroyed<std::string> stats_js(
      LoadResource("drake/geometry/stats.min.js"));
  static const drake::never_destroyed<std::string> meshcat_ico(
      LoadResource("drake/geometry/meshcat.ico"));
  static const drake::never_destroyed<std::string> meshcat_html(
      LoadResource("drake/geometry/meshcat.html"));
  if ((url_path == "/") || (url_path == "/index.html") ||
      (url_path == "/meshcat.html")) {
    return meshcat_html.access();
  }
  if (url_path == "/meshcat.js") {
    return meshcat_js.access();
  }
  if (url_path == "/stats.min.js") {
    return stats_js.access();
  }
  if (url_path == "/favicon.ico") {
    return meshcat_ico.access();
  }
  return {};
}

// We need an Impl for this class for two reasons:
// - The mt19937 object is ginormous and should not be inline.
// - The uuids object must be wrapped within DRAKE_NO_EXPORT.
struct DRAKE_NO_EXPORT UuidGenerator::Impl {
  std::mt19937 prng_;
  uuids::uuid_random_generator uuid_{prng_};
};

UuidGenerator::UuidGenerator() : impl_(std::make_unique<Impl>()) {}

UuidGenerator::~UuidGenerator() = default;

std::string UuidGenerator::GenerateRandom() {
  return uuids::to_string(impl_->uuid_());
}

ScrapedMtllib ScrapeMtllib(const DiagnosticPolicy& diagnostic,
                           std::string_view obj_data,
                           const fs::path& obj_filename) {
  ScrapedMtllib result;

  // Parse the filename out of the "mtllib foo.mtl" line.
  std::string mtllib;
  {
    // TODO(russt): Make this mtllib parsing more robust (right now commented
    // mtllib lines will match, too, etc).
    size_t start = obj_data.find("mtllib ");
    if (start == std::string_view::npos) {
      // No material library.
      return result;
    }
    start += 7;  // Advance to after the actual "mtllib " string.
    size_t finish = obj_data.find('\n', start);
    std::string remainder{obj_data.substr(start, finish)};
    std::smatch matches;
    std::regex_search(remainder, matches, std::regex("\\s*([^\\s]+)"));
    mtllib = matches.str(1);
  }
  result.texture_dir = obj_filename.parent_path();
  const fs::path mtl_filename = result.texture_dir / mtllib;

  // Parse the texture filenames out of the mtllib file. We do parsing manually
  // here: tinyobj does too much work (actually loading all of the content) and
  // also does not give access to the intermediate data that we need to pass to
  // meshcat, like the resource filenames in the mtl file. This is also the
  // approach taken in MeshCat.jl/src/mesh_files.jl.
  std::optional<std::string> mtl_data = ReadFile(mtl_filename);
  if (!mtl_data) {
    diagnostic.Error(fmt::format(
        "Meshcat: When loading \"{}\": Meshcat could not open \"{}\"",
        obj_filename.string(), mtl_filename.string()));
    return result;
  }

  // Scan the .mtl file for `map_...` lines.
  // The syntax (http://paulbourke.net/dataformats/mtl/) is e.g.
  //   map_Ka -options args filename
  // Here we ignore the options and only extract the filename (by
  // extracting the last word before the end of line/string).
  //  - "map_.+" matches the map_ plus any options,
  //  - "\s" matches one whitespace (before the filename),
  //  - "[^\s]+" matches the filename, and
  //  - "[$\r\n]" matches the end of string or end of line.
  // TODO(russt): This parsing could still be more robust.
  std::regex map_regex(R"""(map_.+\s([^\s]+)\s*[$\r\n])""");
  for (std::sregex_iterator iter(mtl_data->begin(), mtl_data->end(), map_regex);
       iter != std::sregex_iterator(); ++iter) {
    std::string map_filename = iter->str(1);
    result.texture_filenames.push_back(std::move(map_filename));
  }

  result.mtl_data = std::move(*mtl_data);

  return result;
}

namespace {

class MeshcatShapeReifier final : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatShapeReifier);

  MeshcatShapeReifier(const DiagnosticPolicy& diagnostic,
                      UuidGenerator* uuid_generator, FileStorage* content,
                      std::vector<FileStorage::Handle>* resources,
                      LumpedObjectData* lumped)
      : diagnostic_(diagnostic),
        uuid_generator_(*uuid_generator),
        content_(*content),
        resources_(*resources),
        lumped_(*lumped) {
    DRAKE_DEMAND(uuid_generator != nullptr);
    DRAKE_DEMAND(content != nullptr);
    DRAKE_DEMAND(resources != nullptr);
    DRAKE_DEMAND(lumped != nullptr);
  }

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box& box, void*) final {
    lumped_.object = MeshData();

    auto geometry = std::make_unique<BoxGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->width = box.width();
    // Three.js uses height for the y axis; Drake uses depth.
    geometry->height = box.depth();
    geometry->depth = box.height();
    lumped_.geometry = std::move(geometry);
  }

  void ImplementGeometry(const Capsule& capsule, void*) final {
    auto& mesh = lumped_.object.emplace<MeshData>();

    auto geometry = std::make_unique<CapsuleGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->radius = capsule.radius();
    geometry->length = capsule.length();
    lumped_.geometry = std::move(geometry);

    // Meshcat cylinders have their long axis in y.
    Eigen::Map<Eigen::Matrix4d>(mesh.matrix) =
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2.0))
            .GetAsMatrix4();
  }

  void ImplementGeometry(const Convex& mesh, void*) final {
    ImplementMesh(mesh.filename(), mesh.extension(), mesh.scale());
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) final {
    auto& mesh = lumped_.object.emplace<MeshData>();

    auto geometry = std::make_unique<CylinderGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->radiusBottom = cylinder.radius();
    geometry->radiusTop = cylinder.radius();
    geometry->height = cylinder.length();
    lumped_.geometry = std::move(geometry);

    // Meshcat cylinders have their long axis in y.
    Eigen::Map<Eigen::Matrix4d>(mesh.matrix) =
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2.0))
            .GetAsMatrix4();
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void*) final {
    // Implemented as a Sphere stretched by a diagonal transform.
    auto& mesh = lumped_.object.emplace<MeshData>();

    auto geometry = std::make_unique<SphereGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->radius = 1;
    lumped_.geometry = std::move(geometry);

    Eigen::Map<Eigen::Matrix4d> matrix(mesh.matrix);
    matrix(0, 0) = ellipsoid.a();
    matrix(1, 1) = ellipsoid.b();
    matrix(2, 2) = ellipsoid.c();
  }

  void ImplementGeometry(const HalfSpace&, void*) final {
    // TODO(russt): Use PlaneGeometry with fields width, height, widthSegments,
    // heightSegments
    diagnostic_.Error("Meshcat does not display HalfSpace geometry (yet).");
  }

  void ImplementGeometry(const Mesh& mesh, void*) final {
    ImplementMesh(mesh.filename(), mesh.extension(), mesh.scale());
  }

  void ImplementGeometry(const MeshcatCone& cone, void*) final {
    auto& mesh = lumped_.object.emplace<MeshData>();

    auto geometry = std::make_unique<CylinderGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->radiusBottom = 0;
    geometry->radiusTop = 1.0;
    geometry->height = cone.height();
    lumped_.geometry = std::move(geometry);

    // Meshcat cylinders have their long axis in y and are centered at the
    // origin.  A cone is just a cylinder with radiusBottom=0.  So we transform
    // here, in addition to scaling to support non-uniform principle axes.
    Eigen::Map<Eigen::Matrix4d>(mesh.matrix) =
        Eigen::Vector4d{cone.a(), cone.b(), 1.0, 1.0}.asDiagonal() *
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2.0),
                        Eigen::Vector3d{0, 0, cone.height() / 2.0})
            .GetAsMatrix4();
  }

  void ImplementGeometry(const Sphere& sphere, void*) final {
    lumped_.object = MeshData();

    auto geometry = std::make_unique<SphereGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->radius = sphere.radius();
    lumped_.geometry = std::move(geometry);
  }

 private:
  // Helper for ImplementGeometry, common to both Mesh and Convex shapes.
  // The `extension` is lowercase and includes the leading dot (e.g., ".obj"),
  // but can also be empty.
  void ImplementMesh(const std::string& filename, const std::string& extension,
                     double scale) {
    // Load the whole file into memory.
    std::optional<std::string> mesh_data = ReadFile(filename);
    if (!mesh_data) {
      diagnostic_.Error(fmt::format(
          "Meshcat: When loading \"{}\": could not open file", filename));
      return;
    }
    if (extension.empty()) {
      diagnostic_.Error(fmt::format(
          "Meshcat: When loading \"{}\": missing filename extension",
          filename));
      return;
    }
    const std::string format = extension.substr(1);
    if (!(format == "obj" || format == "gltf" || format == "dae" ||
          format == "stl")) {
      diagnostic_.Error(fmt::format(
          "Meshcat: When loading \"{}\": unsupported filename extension \"{}\"",
          filename, extension));
      return;
    }

    // For some mesh formats, we also need to load material files.
    // TODO(russt): MeshCat.jl/src/mesh_files.jl loads dae with textures, also.
    ScrapedMtllib scraped_mtllib;
    if (format == "obj") {
      scraped_mtllib = ScrapeMtllib(diagnostic_, *mesh_data, filename);
    }
    const bool mesh_has_materials =
        (format == "gltf") || !scraped_mtllib.mtl_data.empty();

    // Everything loaded okay. Add the mesh data to storage.
    const FileStorage::Handle mesh_handle =
        content_.Insert(std::move(*mesh_data));
    resources_.emplace_back(mesh_handle);
    std::string mesh_url =
        fmt::format("cas/sha256/{}", mesh_handle.sha256.to_string());
    // TODO(russt): Use checksum to generate the uuid, and avoid resending
    // meshes unless necessary.
    std::string uuid = uuid_generator_.GenerateRandom();

    // We need to populate the message very differently depending on whether the
    // mesh file had materials or not.
    if (mesh_has_materials) {
      auto& data = lumped_.object.emplace<MeshfileObjectData>();
      data.format = std::move(format);
      data.url = std::move(mesh_url);
      data.uuid = std::move(uuid);
      data.mtl_library = std::move(scraped_mtllib.mtl_data);
      for (std::string& texture_filename : scraped_mtllib.texture_filenames) {
        const fs::path texture_path =
            scraped_mtllib.texture_dir / texture_filename;
        if (std::optional<std::string> texture_data = ReadFile(texture_path)) {
          FileStorage::Handle texture_handle =
              content_.Insert(std::move(*texture_data));
          std::string texture_url =
              fmt::format("cas/sha256/{}", texture_handle.sha256.to_string());
          data.resources.emplace(std::move(texture_filename),
                                 std::move(texture_url));
          resources_.emplace_back(std::move(texture_handle));
        } else {
          diagnostic_.Error(
              fmt::format("Meshcat: When loading \"{}\": could not open \"{}\"",
                          filename, texture_path.string()));
        }
      }
    } else {
      auto& data = lumped_.emplace_geometry<MeshFileGeometryData>();
      data.format = std::move(format);
      data.url = std::move(mesh_url);
      data.uuid = std::move(uuid);
      lumped_.object.emplace<internal::MeshData>();
    }

    // Set the scale.
    visit_overloaded<void>(
        overloaded{[](std::monostate) {},
                   [scale](auto& lumped_object) {
                     Eigen::Map<Eigen::Matrix4d> matrix(lumped_object.matrix);
                     matrix(0, 0) = scale;
                     matrix(1, 1) = scale;
                     matrix(2, 2) = scale;
                   }},
        lumped_.object);
  }

  const DiagnosticPolicy& diagnostic_;
  UuidGenerator& uuid_generator_;
  FileStorage& content_;
  std::vector<FileStorage::Handle>& resources_;
  LumpedObjectData& lumped_;
};

}  // namespace

void ConvertShapeToMessage(const DiagnosticPolicy& diagnostic,
                           const Shape& shape, UuidGenerator* uuid,
                           FileStorage* content,
                           std::vector<FileStorage::Handle>* resources,
                           LumpedObjectData* lumped) {
  DRAKE_DEMAND(uuid != nullptr);
  DRAKE_DEMAND(content != nullptr);
  DRAKE_DEMAND(resources != nullptr);
  DRAKE_DEMAND(lumped != nullptr);

  *lumped = {};
  MeshcatShapeReifier reifier(diagnostic, uuid, content, resources, lumped);
  shape.Reify(&reifier, nullptr);
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
