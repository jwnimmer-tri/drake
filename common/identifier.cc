#include "drake/common/identifier.h"

#include <atomic>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace internal {

namespace {

// Even though this is a mutable global variable, it does not generate any
// object code for initialization, so it is safe to use even without being
// wrapped within a never_destroyed<>. Note that it is initialized to zero
// at the start of the program via zero-initialization; for details, see
// https://en.cppreference.com/w/cpp/atomic/atomic/atomic.
static std::atomic<int64_t> g_prior_identifier;

}  // namespace

int64_t get_new_identifier()  {
  // Note that 0 is reserved for the uninitialized Identifier created by the
  // default constructor, so we have an invariant that get_new_identifier() > 0.
  // Overflowing an int64_t is not a hazard we need to worry about.
  return ++g_prior_identifier;
}

std::string FormatIdentifier(const std::type_info* type, int64_t value) {
  std::string result;
  if (type != nullptr) {
    std::string full_name = NiceTypeName::Get(*type);
    std::string short_name = NiceTypeName::RemoveNamespaces(full_name);
    if (value > 0) {
      result = fmt::format("{}({})", short_name, value);
    } else {
      result = fmt::format("{}()", short_name);
    }
  } else {
    if (value > 0) {
      result = fmt::to_string(value);
    } else {
      result = "default";
    }
  }
  return result;
}

}  // namespace internal
}  // namespace drake
