#include "drake/common/text_logging.h"

#include <memory>

#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

namespace drake {
namespace logging {
namespace internal {

class LoggerImpl {
 public:
  LoggerImpl() = delete;

  /* Returns a pointer to Drake's singleton "console" logger. */
  static spdlog::logger* console() {
    static const never_destroyed<std::shared_ptr<spdlog::logger>> result(
        MakeLogger());
    return result.access().get();
  }

 private:
  /* Creates Drake's the default logger. NOTE: This function assumes that it is
  mutexed, as in the initializer of a static local. */
  static std::shared_ptr<spdlog::logger> MakeLogger() {
    // Check if anyone has already set up a logger named "console".  If so, we
    // will just return it; if not, we'll create our own default one.
    std::shared_ptr<spdlog::logger> result(spdlog::get("console"));
    if (!result) {
      // We wrap our stderr sink in a dist_sink so that users can atomically
      // swap out the sinks used by all Drake logging, via dist_sink_mt's APIs.
      auto wrapper = std::make_shared<spdlog::sinks::dist_sink_mt>();
      // We use the stderr_sink_mt (instead of stderr_sink_st) so more than one
      // thread can use this logger and have their messages be staggered by
      // line, instead of co-mingling their character bytes.
      wrapper->add_sink(std::make_shared<spdlog::sinks::stderr_sink_mt>());
      result = std::make_shared<spdlog::logger>("console", std::move(wrapper));
      result->set_level(spdlog::level::info);
    }
    logger::level_ = EvilLoggerSubclass::get_level_pointer(result.get());
    return result;
  }

  /* Returns a pointer to the target->level_ member field, using a trick to
  work around protected access controls. */
  struct EvilLoggerSubclass : spdlog::logger {
    static std::atomic_int* get_level_pointer(spdlog::logger* target) {
      return &((*target).*(&EvilLoggerSubclass::level_));
    }
  };
};

}  // namespace internal

using internal::LoggerImpl;

namespace {
// This provides for a non-nullptr default value until drake::log() is called,
// at which point it'll be supplanted by the true pointer.
std::atomic_int dummy_global_atomic_int;
}  // namespace

std::atomic_int* logger::level_ = &dummy_global_atomic_int;

spdlog::sinks::dist_sink_mt* get_dist_sink() {
  // Extract the dist_sink_mt from Drake's logger instance.
  auto& sinks = LoggerImpl::console()->sinks();
  auto* front_sink = sinks.empty() ? nullptr : sinks.front().get();
  auto* result = dynamic_cast<spdlog::sinks::dist_sink_mt*>(front_sink);
  if (result == nullptr) {
    throw std::logic_error(
        "drake::logging::get_sink(): error: the spdlog sink configuration has"
        "unexpectedly changed.");
  }
  return result;
}

void logger::Write(level severity, std::string_view message) {
  LoggerImpl::console()->log(static_cast<spdlog::level::level_enum>(severity),
                             message);
}

void logger::set_level(level severity) {
  LoggerImpl::console()->set_level(
      static_cast<spdlog::level::level_enum>(severity));
}

std::string logger::set_level(std::string_view level) {
  spdlog::level::level_enum prev_value = LoggerImpl::console()->level();
  spdlog::level::level_enum value{};
  if (level == "trace") {
    value = spdlog::level::trace;
  } else if (level == "debug") {
    value = spdlog::level::debug;
  } else if (level == "info") {
    value = spdlog::level::info;
  } else if (level == "warn" || level == "warning") {
    value = spdlog::level::warn;
  } else if (level == "err" || level == "error") {
    value = spdlog::level::err;
  } else if (level == "crit" || level == "critical") {
    value = spdlog::level::critical;
  } else if (level == "off") {
    value = spdlog::level::off;
  } else if (level == "unchanged") {
    value = prev_value;
  } else {
    throw std::runtime_error(fmt::format("Unknown logging level: {}", level));
  }
  LoggerImpl::console()->set_level(value);
  switch (prev_value) {
    case spdlog::level::trace:
      return "trace";
    case spdlog::level::debug:
      return "debug";
    case spdlog::level::info:
      return "info";
    case spdlog::level::warn:
      return "warn";
    case spdlog::level::err:
      return "error";
    case spdlog::level::critical:
      return "critical";
    case spdlog::level::off:
      return "off";
    case spdlog::level::n_levels:
      break;
  }
  DRAKE_UNREACHABLE();
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

void logger::set_pattern(std::string pattern) {
  LoggerImpl::console()->set_pattern(std::move(pattern));
}

const char* const kSetLogPatternHelpMessage =
    "sets the spdlog pattern for formatting; for more information, see "
    "https://github.com/gabime/spdlog/wiki/3.-Custom-formatting";

const char* const kSetLogLevelUnchanged = "unchanged";

}  // namespace logging
}  // namespace drake
