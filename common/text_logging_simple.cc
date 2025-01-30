/* clang-format off */
#include "drake/common/text_logging.h"
/* clang-format on */

#include <atomic>
#include <cstdio>

#include "drake/common/unused.h"

namespace drake {
namespace logging {
namespace {

// Writes the given text to stderr. We use C instead of C++ <iostream> because
// the latter uses static initializers which are forbidded by our style guide.
void WriteToStderr(std::string_view text) {
  const size_t size = 1;
  const size_t count = text.size();
  const size_t written = std::fwrite(text.data(), size, count, stderr);
  unused(written);
}

class logger_impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(logger_impl);
  logger_impl() = default;
  level_enum level() const { return level_.load(std::memory_order_relaxed); }
  void set_level(level_enum severity) { level_ = severity; }
  bool should_log(level_enum severity) const { return severity >= level(); }
  void LogUnconditionally(level_enum /* severity */, std::string_view message) {
    WriteToStderr("drake: ");
    WriteToStderr(message);
    WriteToStderr("\n");
  }

 private:
  std::atomic<level_enum> level_{level::info};
};

}  // namespace

logger::logger() : impl_{new logger_impl} {}

logger::~logger() = default;

bool logger::should_log(level_enum severity) const {
  const auto* const impl = static_cast<const logger_impl*>(impl_);
  return impl->should_log(severity);
}

level_enum logger::level() const {
  const auto* const impl = static_cast<const logger_impl*>(impl_);
  return impl->level();
}

void logger::set_level(level_enum severity) const {
  auto* const impl = static_cast<logger_impl*>(impl_);
  return impl->set_level(severity);
}

void logger::set_pattern(std::string_view /* pattern */) {}

void logger::LogUnconditionally(level_enum severity, std::string_view message) {
  auto* const impl = static_cast<logger_impl*>(impl_);
  return impl->LogUnconditionally(severity, message);
}

}  // namespace logging
}  // namespace drake
