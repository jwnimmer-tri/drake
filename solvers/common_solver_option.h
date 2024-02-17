#pragma once

#include <string>
#include <string_view>

// Remove this include on 2024-09-01 upon completion of deprecation.
#include <ostream>

#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"

namespace drake {
namespace solvers {

/** Some options can be applied to not one solver, but many solvers (for
example, many solvers support printing out the progress in each iteration).
CommonSolverOption contain the names of these supported options. The user can
use these options as "key" in SolverOption::SetOption(). */
enum class CommonSolverOption {
  /** Many solvers support printing the progress of each iteration to a file.
  The user can call SolverOptions::SetOption(kPrintFileName, file_name) where
  file_name is a string. If the user doesn't want to print to a file, then use
  SolverOptions::SetOption(kPrintFileName, ""), where the empty string `""`
  indicates no print. */
  kPrintFileName,

  /** Many solvers support printing the progress of each iteration to the
  console, the user can call `SolverOptions::SetOption(kPrintToConsole, 1)` to
  turn on printing to the console, or use `0` to turn off printing to the
  console. */
  kPrintToConsole,
};

/** Returns the short, unadorned name of the option, e.g., `kPrintFileName`. */
std::string_view to_string(CommonSolverOption);

DRAKE_DEPRECATED("2024-09-01", "Use options.to_string(), instead.")
std::ostream& operator<<(std::ostream&, CommonSolverOption);

namespace internal {

/* Aggregated values for CommonSolverOption, for Drake-internal use only. */
struct CommonSolverOptionValues {
  std::string print_file_name;
  bool print_to_console{false};
};

}  // namespace internal
}  // namespace solvers
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::solvers, CommonSolverOption, x,
                   ::drake::solvers::to_string(x))
