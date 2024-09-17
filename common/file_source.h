#pragma once

#include <filesystem>
#include <variant>

#include "drake/common/memory_file.h"

namespace drake {

using FileSource = std::variant<std::filesystem::path, MemoryFile>;

}  // namespace drake
