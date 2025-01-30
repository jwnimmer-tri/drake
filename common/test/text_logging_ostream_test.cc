/* clang-format off */
#include "drake/common/text_logging.h"
/* clang-format on */

#include <memory>
#include <ostream>

#include <gtest/gtest.h>

#include "drake/common/fmt_ostream.h"

namespace {

class Streamable {
  [[maybe_unused]]  // If we don't have spdlog, this function is dead code.
  friend std::ostream&
  operator<<(std::ostream& os, const Streamable& c) {
    return os << "Hello";
  }
};

using drake::fmt_streamed;

// Checks the interaction between drake::log and drake::fmt_streamed.
// This case is in a separate file so <ostream> doesn't pollute our normal test.
// We call each API function and macro to ensure that all of them compile.
GTEST_TEST(TextLoggingOstreamTest, SmokeTestStreamable) {
  Streamable obj;
  drake::log()->trace("drake::log()->trace test: {} {}", "OK",
                      fmt_streamed(obj));
  drake::log()->debug("drake::log()->debug test: {} {}", "OK",
                      fmt_streamed(obj));
  drake::log()->info("drake::log()->info test: {} {}", "OK", fmt_streamed(obj));
  drake::log()->warn("drake::log()->warn test: {} {}", "OK", fmt_streamed(obj));
  drake::log()->error("drake::log()->error test: {} {}", "OK",
                      fmt_streamed(obj));
  drake::log()->critical("drake::log()->critical test: {} {}", "OK",
                         fmt_streamed(obj));
  DRAKE_LOGGER_TRACE("DRAKE_LOGGER_TRACE macro test: {}, {}", "OK",
                     fmt_streamed(obj));
  DRAKE_LOGGER_DEBUG("DRAKE_LOGGER_DEBUG macro test: {}, {}", "OK",
                     fmt_streamed(obj));
}

}  // namespace
