#include "drake/common/type_safe_index.h"

#include <stdexcept>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace internal {

void ThrowTypeSafeIndexAssertValidFailed(const std::type_info& type,
                                         const char* source) {
  throw std::logic_error(fmt::format(
      "{} Type \"{}\", has an invalid value; it must lie in the range "
      "[0, 2³¹ - 1].",
      source, NiceTypeName::Get(type)));
}

void ThrowTypeSafeIndexAssertNoOverflowFailed(const std::type_info& type,
                                              const char* source) {
  throw std::logic_error(fmt::format("{} Type \"{}\", has overflowed.", source,
                                     NiceTypeName::Get(type)));
}

std::string FormatIndex(const std::type_info* type, int value) {
  std::string result;
  if (type != nullptr) {
    std::string full_name = NiceTypeName::Get(*type);
    std::string short_name = NiceTypeName::RemoveNamespaces(full_name);
    if (value >= 0) {
      result = fmt::format("{}({})", short_name, value);
    } else {
      result = fmt::format("{}()", short_name);
    }
  } else {
    if (value >= 0) {
      result = fmt::to_string(value);
    } else {
      result = "default";
    }
  }
  return result;
}

}  // namespace internal
}  // namespace drake
