#pragma once

#include <filesystem>
#include <string>
#include <variant>

#include "drake/common/memory_file.h"

namespace drake {

/** Represents a file. The file can be on-disk or in-memory. */
using FileSource = std::variant<std::filesystem::path, MemoryFile>;

// TODO(jwnimmer-tri) Deprecate this in lieu of fmt::to_string,
// with advice to include <fmt/std.h>.
/** Returns a string representation. */
std::string to_string(const FileSource& source);

}  // namespace drake
