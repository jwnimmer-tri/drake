/* clang-format off */
#include "drake/common/text_logging.h"
/* clang-format on */

#include <atomic>
#include <iostream>

namespace drake {
namespace logging {
namespace {

class logger_impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(logger_impl);
  logger_impl() = default;
  level_enum level() const { return level_.load(std::memory_order_relaxed); }
  void set_level(level_enum severity) { level_ = severity; }
  bool should_log(level_enum severity) const { return severity >= level(); }
  void log(level_enum /* severity */, std::string_view message) {
    std::cerr << fmt::format("drake: {}\n", message);
  }

 private:
  std::atomic<level_enum> level_{level::off};
};

}  // namespace

logger::logger() : impl_{new logger_impl} {}

logger::~logger() = default;

bool logger::should_log(level_enum severity) const {
  return static_cast<const logger_impl*>(impl_)->should_log(severity);
}

level_enum logger::level() const {
  return static_cast<const logger_impl*>(impl_)->level();
}

void logger::set_level(level_enum severity) const {
  return static_cast<logger_impl*>(impl_)->set_level(severity);
}

void logger::set_pattern(std::string_view /* pattern */) {}

void logger::LogUnconditionally(level_enum severity, std::string_view message) {
  return static_cast<logger_impl*>(impl_)->log(severity, message);
}

}  // namespace logging
}  // namespace drake
