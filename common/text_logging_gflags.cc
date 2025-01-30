#include "drake/common/text_logging_gflags.h"

#include <stdexcept>
#include <string>

#include "drake/common/text_logging.h"

namespace drake {
namespace logging {

std::string set_log_level(const std::string& level) {
  drake::logging::logger* logger = drake::log();
  drake::logging::level_enum prev_value = logger->level();
  drake::logging::level_enum value{};
  if (level == "trace") {
    value = drake::logging::level::trace;
  } else if (level == "debug") {
    value = drake::logging::level::debug;
  } else if (level == "info") {
    value = drake::logging::level::info;
  } else if (level == "warn") {
    value = drake::logging::level::warn;
  } else if (level == "err") {
    value = drake::logging::level::err;
  } else if (level == "critical") {
    value = drake::logging::level::critical;
  } else if (level == "off") {
    value = drake::logging::level::off;
  } else if (level == "unchanged") {
    value = prev_value;
  } else {
    throw std::runtime_error(fmt::format("Unknown log level: {}", level));
  }
  logger->set_level(value);
  switch (prev_value) {
    case drake::logging::level::trace:
      return "trace";
    case drake::logging::level::debug:
      return "debug";
    case drake::logging::level::info:
      return "info";
    case drake::logging::level::warn:
      return "warn";
    case drake::logging::level::err:
      return "err";
    case drake::logging::level::critical:
      return "critical";
    case drake::logging::level::off:
      return "off";
  }
  // For simplicity in linking, we do not use `DRAKE_UNREACHABLE`.
  throw std::runtime_error("Should not reach here!");
}

const char* const kSetLogLevelHelpMessage =
    "sets the spdlog output threshold; possible values are "
    "'unchanged', "
    "'trace', "
    "'debug', "
    "'info', "
    "'warn', "
    "'err', "
    "'critical', "
    "'off'";

const char* const kSetLogLevelUnchanged = "unchanged";

void set_log_pattern(const std::string& pattern) {
  drake::log()->set_pattern(pattern);
}

const char* const kSetLogPatternHelpMessage =
    "sets the spdlog pattern for formatting; for more information, see "
    "https://github.com/gabime/spdlog/wiki/3.-Custom-formatting";

}  // namespace logging
}  // namespace drake
