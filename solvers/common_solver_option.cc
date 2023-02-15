#include "drake/solvers/common_solver_option.h"

#include <stdexcept>

namespace drake {
namespace solvers {
std::string_view to_string(CommonSolverOption value) {
  switch (value) {
    case CommonSolverOption::kPrintFileName:
      return "kPrintFileName";
    case CommonSolverOption::kPrintToConsole:
      return "kPrintToConsole";
  }
  throw std::logic_error("Undefined CommonSolverOption during to_string");
}
}  // namespace solvers
}  // namespace drake
