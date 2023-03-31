/* clang-format off to disable clang-format-includes */
#include "drake/common/text_logging.h"
/* clang-format on */

#include <ostream>
#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/ostream_sink.h>

#include "drake/common/fmt_ostream.h"
#include "drake/common/text_logging_sink.h"

namespace {

class Streamable {
  [[maybe_unused]]  // If we don't have spdlog, this function is dead code.
  friend std::ostream& operator<<(std::ostream& os, const Streamable& c) {
    return os << "OK";
  }
};

using drake::fmt_streamed;

// Call each API function and macro to ensure that all of them compile.
// These should all compile and run both with and without spdlog.
GTEST_TEST(TextLoggingTest, SmokeTestStreamable) {
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

// We must run this test last because it changes the default configuration.
GTEST_TEST(TextLoggingTest, ZZZ_ChangeDefaultSink) {
  // The getter should never return nullptr, even with spdlog disabled.
  spdlog::sinks::dist_sink_mt* const sink = drake::logging::get_dist_sink();
  ASSERT_NE(sink, nullptr);

  // Redirect all logs to a memory stream.
  std::ostringstream messages;
  auto custom_sink = std::make_shared<spdlog::sinks::ostream_sink_st>(
      messages, true /* flush */);
  sink->set_sinks({custom_sink});
  drake::log()->info("This is some good info!");
  EXPECT_THAT(messages.str(), testing::EndsWith(
      "[console] [info] This is some good info!\n"));
}

}  // anon namespace
