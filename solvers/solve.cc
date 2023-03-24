#include "drake/solvers/solve.h"

#include <algorithm>
#include <future>
#include <list>
#include <memory>
#include <thread>
#include <utility>

#include "drake/common/nice_type_name.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace solvers {
MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const std::optional<Eigen::VectorXd>& initial_guess,
    const std::optional<SolverOptions>& solver_options) {
  const SolverId solver_id = ChooseBestSolver(prog);
  drake::log()->debug("solvers::Solve will use {}", solver_id);
  std::unique_ptr<SolverInterface> solver = MakeSolver(solver_id);
  MathematicalProgramResult result{};
  solver->Solve(prog, initial_guess, solver_options, &result);
  return result;
}

MathematicalProgramResult Solve(
    const MathematicalProgram& prog,
    const Eigen::Ref<const Eigen::VectorXd>& initial_guess) {
  const Eigen::VectorXd initial_guess_xd = initial_guess;
  return Solve(prog, initial_guess_xd, {});
}

MathematicalProgramResult Solve(const MathematicalProgram& prog) {
  return Solve(prog, {}, {});
}

namespace {
// Checks if a future has completed execution.
// This function is taken from monte_carlo.cc. It will be used in the "thread
// pool" implementation (which doesn't use the openMP).
template <typename T>
bool IsFutureReady(const std::future<T>& future) {
  // future.wait_for() is the only method to check the status of a future
  // without waiting for it to complete.
  const std::future_status status =
      future.wait_for(std::chrono::milliseconds(1));
  return (status == std::future_status::ready);
}
}  // namespace

std::vector<std::optional<MathematicalProgramResult>> SolveInParallel(
    const std::vector<const MathematicalProgram*>& prog_list,
    const std::vector<EigenPtr<const Eigen::VectorXd>>& initial_guess_list,
    const std::vector<const SolverOptions*>& solver_options_list,
    const int num_threads, const bool terminate_at_first_infeasible) {
  const int num_prog = prog_list.size();
  const int num_initial_guess = initial_guess_list.size();
  const int num_solver_options = solver_options_list.size();
  DRAKE_THROW_UNLESS(num_initial_guess == num_prog || num_initial_guess == 1 ||
                     num_initial_guess == 0);
  DRAKE_THROW_UNLESS(num_solver_options == num_prog ||
                     num_solver_options == 1 || num_solver_options == 0);
  std::vector<std::optional<MathematicalProgramResult>> ret(num_prog);

  auto solve_prog_i = [&](int i) {
    const MathematicalProgram* const prog = prog_list.at(i);
    const EigenPtr<const Eigen::VectorXd> initial_guess =
        num_initial_guess == 0   ? nullptr
        : num_initial_guess == 1 ? initial_guess_list.front()
                                 : initial_guess_list.at(i);
    const SolverOptions* const solver_options =
        num_solver_options == 0   ? nullptr
        : num_solver_options == 1 ? solver_options_list.front()
                                  : solver_options_list.at(i);
    DRAKE_THROW_UNLESS(prog != nullptr);
    const SolverId solver_id = ChooseBestSolver(*prog);
    drake::log()->debug("solvers::SolveInParallel will use {} for i={}",
                        solver_id, i);
    std::unique_ptr<SolverInterface> solver = MakeSolver(solver_id);
    solver->Solve(*prog,
                  initial_guess != nullptr
                      ? std::optional<Eigen::VectorXd>(*initial_guess)
                      : std::nullopt,
                  solver_options != nullptr
                      ? std::optional<SolverOptions>(*solver_options)
                      : std::nullopt,
                  &ret.at(i).emplace());
    return i;
  };

  // We implement the "thread pool" idea here, by following
  // MonteCarloSimulationParallel class. This implementation doesn't use openMP
  // library.
  const int num_threads_actual =
      num_threads > 0
          ? num_threads
          : std::min(static_cast<int>(std::thread::hardware_concurrency()),
                     num_prog);
  std::list<std::future<int>> active_operations;
  // Keep track of how many progs have been dispatched already.
  int progs_dispatched = 0;
  bool stop_dispatching = false;
  while ((active_operations.size() > 0 || (progs_dispatched < num_prog)) &&
         !stop_dispatching) {
    // Check for completed operations.
    for (auto operation = active_operations.begin();
         operation != active_operations.end();) {
      if (IsFutureReady(*operation)) {
        // This call to future.get() is necessary to propagate any exception
        // thrown during the program solve.
        const int prog_count = operation->get();
        drake::log()->info("prog {}/{} completed", prog_count,
                           prog_list.size());

        // Erase returned iterator to the next node in the list.
        operation = active_operations.erase(operation);

        if (terminate_at_first_infeasible &&
            !ret.at(prog_count)->is_success()) {
          stop_dispatching = true;
          break;
        }
      } else {
        // Advance to next node in the list.
        ++operation;
      }
    }

    // Dispatch new prog.
    while (static_cast<int>(active_operations.size()) < num_threads_actual &&
           progs_dispatched < num_prog) {
      active_operations.emplace_back(std::async(
          std::launch::async, std::move(solve_prog_i), progs_dispatched));

      drake::log()->info("prog {}/{} dispatched", progs_dispatched,
                         prog_list.size());

      ++progs_dispatched;
    }
    // Wait a bit before checking for completion.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return ret;
}

}  // namespace solvers
}  // namespace drake
