#include "drake/common/text_logging.h"

#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace {
class Formattable {
 public:
  std::string to_string() const { return "OK"; }
};
}  // namespace

DRAKE_FORMATTER_AS(, , Formattable, x, x.to_string())

namespace drake {
namespace logging {
namespace {

// Call each API function and macro to ensure that all of them compile.
// These should all compile and run both with and without spdlog.
GTEST_TEST(TextLoggingTest, SmokeTestFormattable) {
  Formattable obj;
  drake::log()->trace("drake::log()->trace test: {} {}", "OK", obj);
  drake::log()->debug("drake::log()->debug test: {} {}", "OK", obj);
  drake::log()->info("drake::log()->info test: {} {}", "OK", obj);
  drake::log()->warn("drake::log()->warn test: {} {}", "OK", obj);
  drake::log()->error("drake::log()->error test: {} {}", "OK", obj);
  drake::log()->critical("drake::log()->critical test: {} {}", "OK", obj);
  DRAKE_LOGGER_TRACE("DRAKE_LOGGER_TRACE macro test: {}, {}", "OK", obj);
  DRAKE_LOGGER_DEBUG("DRAKE_LOGGER_DEBUG macro test: {}, {}", "OK", obj);
}

// Check that floating point values format sensibly.  We'll just test fmt
// directly, since we know that spdlog uses it internally.
GTEST_TEST(TextLoggingTest, FloatingPoint) {
  EXPECT_EQ(fmt::format("{}", 1.1), "1.1");
  // This number is particularly challenging.
  EXPECT_EQ(fmt::format("{}", 0.009), "0.009");
}

// Check that the "warn once" idiom compiles and doesn't crash at runtime.
// We use a pattern substitution to cover both arguments of the Warn's ctor.
GTEST_TEST(TextLoggingTest, WarnOnceTest) {
  static const drake::logging::Warn log_once("The {} happened as expected.",
                                             "log_once");
}

// Abuse gtest internals to verify that logging actually prints when enabled,
// and that the default level is INFO.
GTEST_TEST(TextLoggingTest, CaptureOutputTest) {
  testing::internal::CaptureStderr();
  drake::log()->trace("bad sentinel");
  drake::log()->debug("bad sentinel");
  drake::log()->info("good sentinel");
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_THAT(output, testing::HasSubstr("good sentinel"));
  EXPECT_THAT(output, testing::Not(testing::HasSubstr("bad sentinel")));
}

// Verify that DRAKE_LOGGER macros succeed in avoiding evaluation of their
// arguments.
GTEST_TEST(TextLoggingTest, DrakeMacrosDontEvaluateArguments) {
  int tracearg = 0, debugarg = 0;

  // Shouldn't increment argument whether the macro expanded or not, since
  // logging is off.
  drake::log()->set_level(drake::logging::level::off);
  DRAKE_LOGGER_TRACE("tracearg={}", ++tracearg);
  DRAKE_LOGGER_DEBUG("debugarg={}", ++debugarg);
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 0);
  tracearg = 0;
  debugarg = 0;

  // Should increment arg only if the macro expanded.
  drake::log()->set_level(drake::logging::level::trace);
  DRAKE_LOGGER_TRACE("tracearg={}", ++tracearg);
  DRAKE_LOGGER_DEBUG("debugarg={}", ++debugarg);
#ifndef NDEBUG
  EXPECT_EQ(tracearg, 1);
  EXPECT_EQ(debugarg, 1);
#else
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 0);
#endif
  tracearg = 0;
  debugarg = 0;

  // Only DEBUG should increment arg since trace is not enabled.
  drake::log()->set_level(drake::logging::level::debug);
  DRAKE_LOGGER_TRACE("tracearg={}", ++tracearg);
  DRAKE_LOGGER_DEBUG("debugarg={}", ++debugarg);
#ifndef NDEBUG
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 1);
#else
  EXPECT_EQ(tracearg, 0);
  EXPECT_EQ(debugarg, 0);
#endif
  tracearg = 0;
  debugarg = 0;
}

}  // namespace
}  // namespace logging
}  // namespace drake

// To enable compiling without depending on @spdlog, we need to provide our own
// main routine.  The default drake_cc_googletest_main depends on @spdlog.
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
