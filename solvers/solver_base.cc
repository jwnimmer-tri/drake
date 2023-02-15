#include "drake/solvers/solver_base.h"

#include <limits>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"

namespace drake {
namespace solvers {

SolverBase::SolverBase(
    const SolverId& id, std::function<bool()> available,
    std::function<bool()> enabled,
    std::function<bool(const MathematicalProgram&)> are_satisfied,
    std::function<std::string(const MathematicalProgram&)> explain_unsatisfied)
    : solver_id_(id),
      default_available_(std::move(available)),
      default_enabled_(std::move(enabled)),
      default_are_satisfied_(std::move(are_satisfied)),
      default_explain_unsatisfied_(std::move(explain_unsatisfied)) {}

// Remove 2023-06-01 upon completion of deprecation.
SolverBase::SolverBase(
    std::function<SolverId()> id, std::function<bool()> available,
    std::function<bool()> enabled,
    std::function<bool(const MathematicalProgram&)> are_satisfied,
    std::function<std::string(const MathematicalProgram&)> explain_unsatisfied)
    : SolverBase((id != nullptr) ? id() : SolverId{"MISSING"},
                 std::move(available), std::move(enabled),
                 std::move(are_satisfied), std::move(explain_unsatisfied)) {}

SolverBase::~SolverBase() = default;

MathematicalProgramResult SolverBase::Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) const {
  MathematicalProgramResult result;
  this->Solve(prog, initial_guess, solver_options, &result);
  return result;
}

namespace {
std::string ShortName(const SolverInterface& solver) {
  return NiceTypeName::RemoveNamespaces(NiceTypeName::Get(solver));
}
}  // namespace

void SolverBase::Solve(const MathematicalProgram& prog,
                       const std::optional<Eigen::VectorXd>& initial_guess,
                       const std::optional<SolverOptions>& solver_options,
                       MathematicalProgramResult* result) const {
  *result = {};
  if (!available()) {
    const std::string name = ShortName(*this);
    throw std::invalid_argument(fmt::format(
        "{} cannot Solve because {}::available() is false, i.e.,"
        " {} has not been compiled as part of this binary."
        " Refer to the {} class overview documentation for how to compile it.",
        name, name, name, name));
  }
  if (!enabled()) {
    const std::string name = ShortName(*this);
    throw std::invalid_argument(fmt::format(
        "{} cannot Solve because {}::enabled() is false, i.e.,"
        " {} has not been properly configured for use."
        " Typically this means that an environment variable has not been set."
        " Refer to the {} class overview documentation for how to enable it.",
        name, name, name, name));
  }
  if (!AreProgramAttributesSatisfied(prog)) {
    throw std::invalid_argument(ExplainUnsatisfiedProgramAttributes(prog));
  }
  const SolverId id = solver_id();
  result->set_solver_id(id);
  result->set_decision_variable_index(prog.decision_variable_index());
  const Eigen::VectorXd& x_init =
      initial_guess ? *initial_guess : prog.initial_guess();
  if (x_init.rows() != prog.num_vars()) {
    throw std::invalid_argument(
        fmt::format("Solve expects initial guess of size {}, got {}.",
                    prog.num_vars(), x_init.rows()));
  }
  if (!solver_options.has_value() || solver_options->empty()) {
    // The extra options are empty, so we can just use the prog options.
    DoSolve(prog, x_init, prog.solver_options(), result);
  } else if (prog.solver_options().empty()) {
    // The prog options are empty, so we can just use the extra options.
    DoSolve(prog, x_init, *solver_options, result);
  } else {
    // Both sets of options are non-empty, so we need to merge them.
    SolverOptions merged(prog.solver_options());
    merged.Update(*solver_options, internal::GetCommonOptionPseudoSolverId());
    merged.Update(*solver_options, id);
    DoSolve(prog, x_init, merged, result);
  }
}

bool SolverBase::available() const {
  DRAKE_DEMAND(default_available_ != nullptr);
  return default_available_();
}

bool SolverBase::enabled() const {
  DRAKE_DEMAND(default_enabled_ != nullptr);
  return default_enabled_();
}

// On 2023-06-01 upon completion of deprecation, move this function definition
// to the header file and change it from `override` to `final`.
SolverId SolverBase::solver_id() const {
  return solver_id_;
}

bool SolverBase::AreProgramAttributesSatisfied(
    const MathematicalProgram& prog) const {
  DRAKE_DEMAND(default_are_satisfied_ != nullptr);
  return default_are_satisfied_(prog);
}

std::string SolverBase::ExplainUnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) const {
  if (default_explain_unsatisfied_ != nullptr) {
    return default_explain_unsatisfied_(prog);
  }
  if (AreProgramAttributesSatisfied(prog)) {
    return {};
  }
  return fmt::format(
      "{} is unable to solve a MathematicalProgram with {}.",
      ShortName(*this), to_string(prog.required_capabilities()));
}

}  // namespace solvers
}  // namespace drake
