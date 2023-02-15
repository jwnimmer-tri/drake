#pragma once

#include <ostream>

#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"

namespace drake {
namespace solvers {
/** Some options can be applied many solvers, not just one (e.g., many solvers
support printing out the progress in each iteration). CommonSolverOption denotes
the names of these supported options. The user can use these options as "key" in
SolverOptions::SetOption(). */
enum class CommonSolverOption {
  /** Many solvers support printing the progress of each iteration to a file.
  The user can call SolverOptions::SetOption(kPrintFileName, file_name) where
  file_name is a string. If the user doesn't want to print to a file, then use
  SolverOptions::SetOption(kPrintFileName, ""), where the empty string ""
  indicates no print. */
  kPrintFileName,

  /** Many solvers support printing the progress of each iteration to the
  console. The user can call SolverOptions::SetOption(kPrintToConsole, 1) to
  turn on printing to the console, or 0 to turn off printing to the console. */
  kPrintToConsole,
};

/** Returns the string name like "kPrintFileName" for the given option. */
std::string_view to_string(CommonSolverOption);

// TODO(jwnimmer-tri) On 2023-06-01 also remove the <ostream> include above.
DRAKE_DEPRECATED("2023-06-01",
    "Use fmt or spdlog for logging, not operator<<. "
    "See https://github.com/RobotLocomotion/drake/issues/17742 for details.")
inline std::ostream& operator<<(std::ostream& os, CommonSolverOption value) {
  os << to_string(value);
  return os;
}

}  // namespace solvers
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::solvers, CommonSolverOption, x, to_string(x))
