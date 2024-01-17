#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "drake/common/diagnostic_policy.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/meshcat_types_internal.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace geometry {
namespace internal {

/* Returns the static content for the given URL, or nullopt when the URL is
invalid. The valid static resource URLs are:
- `/`
- `/favicon.ico`
- `/index.html`
- `/meshcat.html`
- `/meshcat.js`
- `/stats.min.js` */
std::optional<std::string_view> GetMeshcatStaticResource(
    std::string_view url_path);

/* UuidGenerator generates random UUIDs:
https://en.wikipedia.org/wiki/Universally_unique_identifier#Version_4_(random)

This object is stateful so that each UUID will be distinct; the intended use is
to create one long-lived instance that services all requests for the lifetime of
the process.

Note that the UUIDs are *deterministically* random -- the i'th random UUID will
be the same from one run to the next. There is no re-seeding. */
class UuidGenerator final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UuidGenerator);

  UuidGenerator();
  ~UuidGenerator();

  /* Returns a newly-generated random UUID. */
  std::string GenerateRandom();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/* The return type for ScrapeMtllib. */
struct ScrapedMtllib {
  /* The contents of the `*.mtl` file. */
  std::string mtl_data;
  /* The list of filenames (if any) mentioned in the `mtl_data`. */
  std::vector<std::string> texture_filenames;
  /* The directory that `texture_filenames` should be taken relative to. */
  std::filesystem::path texture_dir;
};

/* Given the contents of a `*.obj` file as `obj_data` and its path as
`obj_path`, returns the content of its associated `*.mtl` file and the texture
filenames reference by the `*.mtl` content. On error, the `result.mtl_data` will
be empty. */
ScrapedMtllib ScrapeMtllib(const drake::internal::DiagnosticPolicy& diagnostic,
                           const std::string_view obj_data,
                           const std::filesystem::path& obj_path);

/* Populates `lumped` to reflect the given `shape`.
All of the arguments must be non-null.
@param[in,out] content Cache used to store the large data buffers.
@param[in,out] resources Any items added to the cache will be appended here.
@param[out] lumped The output object to be overwritten.
*/
void ConvertShapeToMessage(const drake::internal::DiagnosticPolicy& diagnostic,
                           const Shape& shape, UuidGenerator* uuid,
                           FileStorage* content,
                           std::vector<FileStorage::Handle>* resources,
                           LumpedObjectData* lumped);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
