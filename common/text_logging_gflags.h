#pragma once

/** @file
XXX ... */

#include <string>

namespace drake {
namespace logging {

/** Sets the log threshold used by Drake's C++ code.
@param level Must be either one of the level values as a string ("trace",
"debug", "info", "warn", "err", "critical", "off"), or else the constant
kSetLogLevelUnchanged to request no change.
@return The string value of the previous log level. */
std::string set_log_level(const std::string& level);

/** The "unchanged" string to pass to set_log_level() so as to achieve a
no-op. */
extern const char* const kSetLogLevelUnchanged;

/** An end-user help string suitable to describe the effects of
set_log_level(). */
extern const char* const kSetLogLevelHelpMessage;

/** Invokes `drake::log()->set_pattern(pattern)`.
This has no effect if spdlog is disabled.
@param pattern Formatting for message. For more information, see:
https://github.com/gabime/spdlog/wiki/3.-Custom-formatting */
void set_log_pattern(const std::string& pattern);

/** An end-user help string suitable to describe the effects of
set_log_pattern(). */
extern const char* const kSetLogPatternHelpMessage;

}  // namespace logging
}  // namespace drake
