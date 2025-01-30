#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/text_logging.h"

namespace drake {
namespace logging {
namespace {

GTEST_TEST(TextLoggingGflagsTest, SetLogLevel) {
  EXPECT_THROW(set_log_level("bad"), std::runtime_error);
  const std::vector<std::string> levels = {"trace", "debug",    "info", "warn",
                                           "err",   "critical", "off"};
  const std::string first_level = set_log_level("unchanged");
  std::string prev_level = "off";
  set_log_level(prev_level);
  for (const std::string& level : levels) {
    EXPECT_EQ(set_log_level(level), prev_level);
    prev_level = level;
  }
  set_log_level(first_level);
}

GTEST_TEST(TextLoggingGflagsTest, SetLogPattern) {
  set_log_pattern("%v");
  set_log_pattern("%+");
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
