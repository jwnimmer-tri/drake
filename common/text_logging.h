#pragma once

/** @file
This is the entry point for all text logging within Drake. Once you've
included this file, the suggested ways you should write log messages
include:

<pre>
  drake::log()->trace("Some trace message: {} {}", something, some_other);
</pre>
Similarly, it provides:
<pre>
  drake::log()->debug(...);
  drake::log()->info(...);
  drake::log()->warn(...);
  drake::log()->error(...);
  drake::log()->critical(...);
</pre>
If you want to log objects that are expensive to serialize, these macros will
not be compiled if debugging is turned off (-DNDEBUG is set):
<pre>
  DRAKE_LOGGER_TRACE("message: {}", something_conditionally_compiled);
  DRAKE_LOGGER_DEBUG("message: {}", something_conditionally_compiled);
</pre>

The format string syntax is fmtlib; see https://fmt.dev/latest/syntax.html.
(Note that the documentation link provides syntax for the latest version of
fmtlib; the version of fmtlib used by Drake might be older.)

When formatting an Eigen matrix into a string you must wrap the Eigen object
with fmt_eigen(); see its documentation for details. This holds true whether it
be for logging, error messages, etc.

When logging a third-party type whose only affordance for string output is
`operator<<`, use fmt_streamed(); see its documentation for details. This is
very rare (only a couple uses in Drake so far).

When implementing a string output for a Drake type, eventually this page will
demonstrate how to use fmt::formatter<T>. In the meantime, you can implement
`operator<<` and use drake::ostream_formatter, or else use the macro helper
DRAKE_FORMATTER_AS(). Grep around in Drake's existing code to find examples. */

#include <atomic>
#include <string>
#include <type_traits>
#include <utility>

// TODO(jwnimmer-tri) After deprecation date 2023-09-01 expires, remove this
// backwards-compatibility include statement.
#include <spdlog/spdlog.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"

// Note to maintainers: some of the class names in this file are snake_case for
// backwards compatibility with spdlog, instead of CamelCase per GSG.

namespace drake {

#ifndef DRAKE_DOXYGEN_CXX
namespace logging {
class logger;
namespace internal {
class LoggerImpl;
}  // namespace internal
}  // namespace logging
#endif

/** Retrieves the object used by Drake for text logging; for example:
<pre>
  drake::log()->info("Found {} solutions", num_solutions);
</pre>
See the text_logging.h documentation for a short tutorial. */
logging::logger* log();

namespace logging {

/** Severity thresholds for logging. The names are chosen to align with
https://github.com/gabime/spdlog and similar logging packages. */
enum class level : int {
  trace = 0,
  debug = 1,
  info = 2,
  warn = 4,
  error = 4,
  critical = 5,
  off = 6,
  // Also add some compatiblity synonyms.
  warning = warn,
  err = error,
  crit = critical,
};

/** The entry point for posting a log message. Use drake::log() to get a pointer
to the singleton instance of this class. */
class logger final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(logger)

  // Add sugar for level-specific logging. Provide a Doxygen summary without all
  // of the template boilerplate, and then use a macro to emit the boilerplate.
#ifdef DRAKE_DOXYGEN_CXX
  /** Logs a formatted message at level::trace. If there are no args, then the
  format_string is logged as a string literal (not a format string). */
  template <typename FormatString, typename... Args>
  void trace(Format format_string, Args&&... args);
  /** Logs a formatted message at level::debug. If there are no args, then the
  format_string is logged as a string literal (not a format string). */
  template <typename FormatString, typename... Args>
  void debug(Format format_string, Args&&... args);
  /** Logs a formatted message at level::info. If there are no args, then the
  format_string is logged as a string literal (not a format string). */
  template <typename FormatString, typename... Args>
  void info(Format format_string, Args&&... args);
  /** Logs a formatted message at level::warn. If there are no args, then the
  format_string is logged as a string literal (not a format string). */
  template <typename FormatString, typename... Args>
  void warn(Format format_string, Args&&... args);
  /** Logs a formatted message at level::error. If there are no args, then the
  format_string is logged as a string literal (not a format string). */
  template <typename FormatString, typename... Args>
  void error(Format format_string, Args&&... args);
  /** Logs a formatted message at level::crit. If there are no args, then the
  format_string is logged as a string literal (not a format string). */
  template <typename FormatString, typename... Args>
  void crit(Format format_string, Args&&... args);
#else
#define DRAKE_LOGGER_LOG_AT(function_name)                                 \
  template <typename T>                                                    \
  void function_name(const T& message) {                                   \
    this->Log(level::function_name, "{}", std::string_view{message});      \
  }                                                                        \
  template <typename Arg, typename... Args>                                \
  void function_name(fmt::format_string<Arg, Args...> format_string,       \
                     Arg&& arg, Args&&... args) {                          \
    this->Log(level::function_name, format_string, std::forward<Arg>(arg), \
              std::forward<Args>(args)...);                                \
  }
  DRAKE_LOGGER_LOG_AT(trace)
  DRAKE_LOGGER_LOG_AT(debug)
  DRAKE_LOGGER_LOG_AT(info)
  DRAKE_LOGGER_LOG_AT(warn)
  DRAKE_LOGGER_LOG_AT(error)
  DRAKE_LOGGER_LOG_AT(critical)
#undef DRAKE_LOGGER_LOG_AT
#endif

#ifndef DRAKE_DOXYGEN_CXX
  /* (Internal use only) Returns the current log level as an int. */
  static int internal_get_level() {
    return level_->load(std::memory_order_relaxed);
  }
#endif

  /** Sets the log threshold for Drake's logging. */
  void set_level(level severity);

  /** Sets the log threshold for Drake's logging.
  @param level Must be either one of drake::logging::level enum names ("debug",
  "info", etc.) or else "unchanged".
  @returns The string value of the previous log level. */
  std::string set_level(std::string_view level);

  /** Changes the formatting patter for Drake text logs.
  @param pattern Formatting for messages. For more information, see:
  https://github.com/gabime/spdlog/wiki/3.-Custom-formatting */
  void set_pattern(std::string pattern);

  DRAKE_DEPRECATED("2023-09-01", "Use drake::logging::level not spdlog.")
  std::string set_level(spdlog::level::level_enum);

  DRAKE_DEPRECATED("2023-09-01", "The logger no longe has a name.")
  std::string name() const {
    return "console";
  }

 private:
  // Allow log() to access our singleton instance.
  friend logger* drake::log();

  // Allow implementation details in text_logging.cc to access private members.
  friend class internal::LoggerImpl;

  /* Trivial private constructor for use by instance(). */
  logger() = default;

  /* Returns the logger singleton; see drake::log() for the public wrapper. */
  static logger* instance() {
    static_assert(std::is_trivial_v<logger>);
    static logger singleton;
    return &singleton;
  }

  /* If the given `severity` is enabled, substitutes `args` into `format_string`
  and logs the resulting string. */
  template <typename... Args>
  void Log(level severity, fmt::format_string<Args...> format_string,
           Args&&... args) {
    if (static_cast<int>(severity) >= internal_get_level()) {
      Write(severity, fmt::format(format_string, std::forward<Args>(args)...));
    }
  }

  /* Writes the given message to the log. */
  static void Write(level severity, std::string_view message);

  /* Pointer to Drake's global log level. */
  static std::atomic_int* level_;
};

}  // namespace logging

/** Retrieve an instance of a logger to use for logging; for example:
<pre>
  drake::log()->info("Found {} solutions", num_solutions);
</pre>
See the text_logging.h documentation for a short tutorial. */
inline logging::logger* log() {
  return logging::logger::instance();
}

namespace logging {

/** When constructed, logs a message (at "warn" severity); the destructor is
guaranteed to be trivial.  This is useful for declaring an instance of this
class as a function-static global, so that a warning is logged the first time
the program encounters some code, but does not repeat the warning on subsequent
encounters within the same process.

For example:
<pre>
double* SanityCheck(double* data) {
  if (!data) {
    static const logging::Warn log_once("Bad data!");
    return alternative_data();
  }
  return data;
}
</pre> */
struct Warn {
  // Overload for 1 argument.
  template <typename T>
  explicit Warn(const T& message) {
    drake::log()->warn(message);
  }
  // Overload for >= 2 arguments.
  template <typename Arg, typename... Args>
  Warn(fmt::format_string<Arg, Args...> format_string, Arg&& arg,
       Args&&... args) {
    drake::log()->warn(format_string, std::forward<Arg>(arg),
                       std::forward<Args>(args)...);
  }
};

/** (Advanced) The "unchanged" string to pass to drake::log()->set_level() so as
to achieve a no-op. This is typically only used for configuration functions such
as gflags support. */
extern const char* const kSetLogLevelUnchanged;

/** (Advanced) An end-user help string suitable to describe the effects of
drake::log()->set_level(). This is typically only used for configuration
functions such as gflags support. */
extern const char* const kSetLogLevelHelpMessage;

/** (Advanced) An end-user help string suitable to describe the effects of
drake::log()->set_pattern(). This is typically only used for configuration
functions such as gflags support. */
extern const char* const kSetLogPatternHelpMessage;

}  // namespace logging
}  // namespace drake

#if !defined(NDEBUG) && !defined(DRAKE_DOXYGEN_CXX)

// Provide operative macros only when Debug is enabled.
#define DRAKE_LOGGER_TRACE(...)                                              \
  do {                                                                       \
    /* Capture the drake::log() in a temporary, using a relatively unique */ \
    /* variable name to avoid potential variable name shadowing warnings. */ \
    ::drake::logging::logger* const drake_spdlog_macro_logger_alias =        \
        ::drake::log();                                                      \
    if (::drake::logging::logger::internal_get_level() <=                    \
        static_cast<int>(::drake::logging::level::trace)) {                  \
      drake_spdlog_macro_logger_alias->trace(__VA_ARGS__);                   \
    }                                                                        \
  } while (0)
#define DRAKE_LOGGER_DEBUG(...)                                              \
  do {                                                                       \
    /* Capture the drake::log() in a temporary, using a relatively unique */ \
    /* variable name to avoid potential variable name shadowing warnings. */ \
    ::drake::logging::logger* const drake_spdlog_macro_logger_alias =        \
        ::drake::log();                                                      \
    if (::drake::logging::logger::internal_get_level() <=                    \
        static_cast<int>(::drake::logging::level::debug)) {                  \
      drake_spdlog_macro_logger_alias->debug(__VA_ARGS__);                   \
    }                                                                        \
  } while (0)

#else

// We are doing a non-Debug build.
#define DRAKE_LOGGER_TRACE(...)
#define DRAKE_LOGGER_DEBUG(...)

#endif  // !NDEBUG && !DRAKE_DOXYGEN_CXX

// TODO(jwnimmer-tri) After deprecation date 2023-09-01 expires, remove this
// backwards-compatibility include statement.
#include "drake/common/text_logging_sink.h"

// TODO(jwnimmer-tri) After deprecation date 2023-09-01 expires, remove the
// entire following namespace and contents.
namespace drake {
namespace logging {

DRAKE_DEPRECATED("2023-09-01", "This is always true now.")
constexpr bool kHaveSpdlog = true;

using sink DRAKE_DEPRECATED("2023-09-01", "Replace with spdlog::sinks::sink.") =
    spdlog::sinks::sink;

DRAKE_DEPRECATED("2023-09-01", "Use drake::log()->set_pattern() instead.")
inline void set_log_pattern(const std::string& pattern) {
  drake::log()->set_pattern(pattern);
}

DRAKE_DEPRECATED("2023-09-01", "Use drake::log()->set_level() instead.")
inline std::string set_log_level(const std::string& level) {
  return drake::log()->set_level(level);
}

}  // namespace logging
}  // namespace drake
