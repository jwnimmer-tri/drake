// This file contains the implementation of both drake_assert and drake_throw.
/* clang-format off to disable clang-format-includes */
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
/* clang-format on */

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <fmt/format.h>

#include "drake/common/drake_assertion_error.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace internal {
namespace {

// Singleton to manage assertion configuration.
struct AssertionConfig {
  static AssertionConfig& singleton() {
    static never_destroyed<AssertionConfig> global;
    return global.access();
  }

  std::atomic<bool> assertion_failures_are_exceptions;
};

// Combines the given failure details into a cohesive string message and
// returns it. Of all the input arguments, only `condition` may be null.
std::string FormatFailureDetail(const char* condition, const char* func,
                                const char* file, int line) {
  return fmt::format("Failure at {}:{} in {}(){}.", file, line, func,
                     (condition != nullptr)
                         ? fmt::format(": condition '{}' failed", condition)
                         : std::string{});
}
}  // namespace

// Declared in drake_assert.h.
void Abort(const char* condition, const char* func, const char* file,
           int line) {
  const std::string detail = FormatFailureDetail(condition, func, file, line);
  // We use fprintf instead of std::cerr to avoid C++ static constructor and
  // destructor shenanigans that come from including <iostream>.
  std::fprintf(stderr, "abort: %s\n", detail.c_str());
  std::abort();
}

// Declared in drake_throw.h.
void Throw(const char* condition, const char* func, const char* file,
           int line) {
  throw assertion_error(FormatFailureDetail(condition, func, file, line));
}

// Declared in drake_assert.h.
void AssertionFailed(const char* condition, const char* func, const char* file,
                     int line) {
  if (AssertionConfig::singleton().assertion_failures_are_exceptions) {
    Throw(condition, func, file, line);
  } else {
    Abort(condition, func, file, line);
  }
}

}  // namespace internal
}  // namespace drake

// Configures the DRAKE_ASSERT and DRAKE_DEMAND assertion failure handling
// behavior.
//
// By default, assertion failures will result in an ::abort().  If this method
// has ever been called, failures will result in a thrown exception instead.
//
// Assertion configuration has process-wide scope.  Changes here will affect
// all assertions within the current process.
//
// This method is intended ONLY for use by pydrake bindings, and thus is
// declared here in the cc file (not in any header file), to discourage
// anyone from using it.
extern "C" void drake_set_assertion_failure_to_throw_exception();

// Define the function (separate from declaration) to avoid compiler warnings.
void drake_set_assertion_failure_to_throw_exception() {
  drake::internal::AssertionConfig::singleton()
      .assertion_failures_are_exceptions = true;
}
