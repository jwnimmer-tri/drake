#include "drake/solvers/choose_best_solver.h"

#include <array>
#include <string>
#include <unordered_map>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"
#include "drake/solvers/clarabel_solver.h"
#include "drake/solvers/clp_solver.h"
#include "drake/solvers/csdp_solver.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/get_program_type.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/linear_system_solver.h"
#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/nlopt_solver.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/scs_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace solvers {
namespace {

// A collection of function pointers to the static member functions that are
// twins to the SolverInterface virtual member functions.  These pointers are
// useful when we want to interrogate solvers without constructing them.
class StaticSolverInterface {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StaticSolverInterface)

  template <typename SomeSolver>
  static constexpr StaticSolverInterface Make() {
    StaticSolverInterface result;
    result.id_ = &SomeSolver::id;
    result.is_available_ = &SomeSolver::is_available;
    result.is_enabled_ = &SomeSolver::is_enabled;
    result.are_program_attributes_satisfied_ =
        &SomeSolver::ProgramAttributesSatisfied;
    result.make_ = &MakeAndUpcast<SomeSolver>;
    return result;
  }

  SolverId id() const { return (*id_)(); }
  bool is_available() const { return (*is_available_)(); }
  bool is_enabled() const { return (*is_enabled_)(); }
  bool AreProgramAttributesSatisfied(const MathematicalProgram& prog) const {
    return (*are_program_attributes_satisfied_)(prog);
  }
  std::unique_ptr<SolverInterface> Make() const { return (*make_)(); }

 private:
  StaticSolverInterface() = default;

  // A helper function that combines make_unique with an upcast.
  template <typename SomeSolver>
  static std::unique_ptr<SolverInterface> MakeAndUpcast() {
    return std::make_unique<SomeSolver>();
  }

  SolverId (*id_)() = nullptr;
  bool (*is_available_)() = nullptr;
  bool (*is_enabled_)() = nullptr;
  bool (*are_program_attributes_satisfied_)(const MathematicalProgram& prog) =
      nullptr;
  std::unique_ptr<SolverInterface> (*make_)() = nullptr;
};

// XXX
template <typename Solver>
constexpr auto kStaticSolver = StaticSolverInterface::Make<Solver>();

// The list of all solvers compiled in Drake.
constexpr std::array<const StaticSolverInterface*, 13> kKnownSolvers{
    &kStaticSolver<ClarabelSolver>,
    &kStaticSolver<ClpSolver>,
    &kStaticSolver<CsdpSolver>,
    &kStaticSolver<EqualityConstrainedQPSolver>,
    &kStaticSolver<GurobiSolver>,
    &kStaticSolver<IpoptSolver>,
    &kStaticSolver<LinearSystemSolver>,
    &kStaticSolver<MobyLCPSolver<double>>,
    &kStaticSolver<MosekSolver>,
    &kStaticSolver<NloptSolver>,
    &kStaticSolver<OsqpSolver>,
    &kStaticSolver<ScsSolver>,
    &kStaticSolver<SnoptSolver>};

constexpr int kNumberOfSolvers = kKnownSolvers.size();

// The list of all solvers compiled in Drake, indexed by SolverId.
using KnownSolversMap =
    std::unordered_map<SolverId, const StaticSolverInterface*>;
const KnownSolversMap& GetKnownSolversMap() {
  static const never_destroyed<KnownSolversMap> result{[]() {
    KnownSolversMap prototype;
    for (const auto* solver : kKnownSolvers) {
      prototype.emplace(solver->id(), solver);
    }
    return prototype;
  }()};
  return result.access();
}

// Returns the list of solvers which can solve the given program type.
// If conservative is true, outputs the list that can _always_ solve
// any program of that the type; if false, outputs the list that can
// solve _at least one_ instance of the type, but maybe not all.
constexpr std::array<const StaticSolverInterface*, kNumberOfSolvers>
GetSolversFor(ProgramType prog_type, bool conservative) {
  std::array<const StaticSolverInterface*, kNumberOfSolvers> result = {};
  auto iter = result.data();
  switch (prog_type) {
    case ProgramType::kLP: {
      if (!conservative) {
        // LinearSystemSolver solves a specific type of problem A*x=b, we
        // put the more specific solver ahead of other general solvers.
	*iter++ = &kStaticSolver<LinearSystemSolver>;
      }
      // Preferred solvers.
      // The order between Mosek and Gurobi can be changed. According to the
      // benchmark http://plato.asu.edu/bench.html, Gurobi is slightly faster
      // than Mosek (as of 07-26-2021), but the difference is not significant.
      // Clp is slower than Gurobi and Mosek (with the barrier method).
      // Clarabel is also a very good open-source solver. I found that Clarabel
      // is slightly less accurate than Clp.
      *iter++ = &kStaticSolver<GurobiSolver>;
      *iter++ = &kStaticSolver<MosekSolver>;
      *iter++ = &kStaticSolver<ClpSolver>;
      *iter++ = &kStaticSolver<ClarabelSolver>;
      // Dispreferred (generic nonlinear solvers).
      // I generally find SNOPT faster than IPOPT. Nlopt is less reliable.
      *iter++ = &kStaticSolver<SnoptSolver>;
      *iter++ = &kStaticSolver<IpoptSolver>;
      *iter++ = &kStaticSolver<NloptSolver>;
      // Dispreferred (cannot handle free variables).
      *iter++ = &kStaticSolver<CsdpSolver>;
      // Dispreferred (ADMM, low accuracy).
      *iter++ = &kStaticSolver<ScsSolver>;
      return result;
    }
    case ProgramType::kQP: {
      if (!conservative) {
        // EqualityConstrainedQPSolver solves a specific QP with only linear
        // equality constraints. We put the more specific solver ahead of the
        // general solvers.
        *iter++ = &kStaticSolver<EqualityConstrainedQPSolver>;
      }
      // Preferred solvers.
      // According to http://plato.asu.edu/ftp/cconvex.html, Mosek is slightly
      // faster than Gurobi. In practice, I find their performance comparable,
      // so the order between these two can be switched.
      *iter++ = &kStaticSolver<MosekSolver>;
      *iter++ = &kStaticSolver<GurobiSolver>;
      // Clarabel is an open-source QP solver with relatively good accuracy.
      // (But might not be as fast as OSQP).
      *iter++ = &kStaticSolver<ClarabelSolver>;
      // Dispreferred (ADMM, low accuracy).
      // Although both OSQP and SCS use ADMM, I find OSQP to be more accurate
      // than SCS. Oftentimes I find OSQP generates solution with reasonable
      // accuracy, so I put it before SNOPT/IPOPT/NLOPT (which are often slower
      // than OSQP).
      *iter++ = &kStaticSolver<OsqpSolver>;
      // TODO(hongkai.dai): add CLP to this list when we resolve the memory
      // issue in CLP. 
      // Dispreferred (generic nonlinear solvers). I find SNOPT often faster
      // than IPOPT. NLOPT is less reliable.
      *iter++ = &kStaticSolver<SnoptSolver>;
      *iter++ = &kStaticSolver<NloptSolver>;
      *iter++ = &kStaticSolver<NloptSolver>;
      // Dispreferred (ADMM, low accuracy).
      *iter++ = &kStaticSolver<ScsSolver>;
      return result;
    }
    case ProgramType::kSOCP: {
      // Preferred solvers.
      // According to http://plato.asu.edu/ftp/socp.html, Mosek is slightly
      // faster than Gurobi for SOCP, but the difference is small.
      *iter++ = &kStaticSolver<MosekSolver>;
      *iter++ = &kStaticSolver<GurobiSolver>;
      // ClarabelSolver is a good open-source convex solver.
      *iter++ = &kStaticSolver<ClarabelSolver>;
      // Dispreferred (cannot handle free variables).
      *iter++ = &kStaticSolver<CsdpSolver>;
      // Dispreferred (ADMM, low accuracy).
      *iter++ = &kStaticSolver<ScsSolver>;
      // Dispreferred (generic nonlinear solvers).
      // I strongly suggest NOT to use these solvers for SOCP. These gradient-
      // based solvers ignore the convexity of the problem, and also these
      // solvers require smooth gradient, while the second-order cone constraint
      // is not differentiable at the tip of the cone.
      *iter++ = &kStaticSolver<SnoptSolver>;
      *iter++ = &kStaticSolver<IpoptSolver>;
      *iter++ = &kStaticSolver<NloptSolver>;
      return result;
    }
    case ProgramType::kSDP: {
      // Preferred solvers.
      *iter++ = &kStaticSolver<MosekSolver>;
      // Clarabel is a good open-source convex solver. Preferred.
      *iter++ = &kStaticSolver<ClarabelSolver>;
      // Dispreferred (cannot handle free variables).
      *iter++ = &kStaticSolver<CsdpSolver>;
      // Dispreferred (ADMM, low accuracy).
      *iter++ = &kStaticSolver<ScsSolver>;
      // Dispreferred (generic nonlinear solvers). I strongly suggest NOT to
      // use these solvers for SDP. These gradient-based solvers ignore the
      // convexity of the problem, and also these solvers require smooth
      // gradient, while the semidefinite cone constraint is not differentiable
      // everywhere (when the minimal eigen value is not unique).
      if (!conservative) {
        *iter++ = &kStaticSolver<SnoptSolver>;
        *iter++ = &kStaticSolver<IpoptSolver>;
        *iter++ = &kStaticSolver<NloptSolver>;
      }
      return result;
    }
    case ProgramType::kGP:
    case ProgramType::kCGP: {
      // TODO(hongkai.dai): These types of programs should not be called GP or
      // CGP. GP are programs with posynomial constraints. Find the right name
      // of these programs with exponential cones, and add to the documentation
      // in https://drake.mit.edu/doxygen_cxx/group__solvers.html
      // Preferred solver.
      *iter++ = &kStaticSolver<MosekSolver>;
      // Open-source preferred solver.
      *iter++ = &kStaticSolver<ClarabelSolver>;
      // Dispreferred solver, low accuracy (with ADMM method).
      *iter++ = &kStaticSolver<ScsSolver>;
      return result;
    }
    case ProgramType::kMILP:
    case ProgramType::kMIQP:
    case ProgramType::kMISOCP: {
      // According to http://plato.asu.edu/ftp/misocp.html, Gurobi is a lot
      // faster than Mosek for MISOCP. The benchmark doesn't compare Mosek
      // against Gurobi for MILP and MIQP.
      *iter++ = &kStaticSolver<GurobiSolver>;
      *iter++ = &kStaticSolver<MosekSolver>;
      return result;
    }
    case ProgramType::kMISDP: {
      return result;
    }
    case ProgramType::kQuadraticCostConicConstraint: {
      // Preferred solvers.
      // I don't know if Mosek is better than Gurobi for this type of programs.
      *iter++ = &kStaticSolver<MosekSolver>;
      *iter++ = &kStaticSolver<GurobiSolver>;
      // Open-source preferred solver.
      *iter++ = &kStaticSolver<ClarabelSolver>;
      // Dispreferred solver (ADMM, low accuracy).
      *iter++ = &kStaticSolver<ScsSolver>;
      return result;
    }
    case ProgramType::kNLP: {
      // I find SNOPT often faster than IPOPT. NLOPT is less reliable.
      *iter++ = &kStaticSolver<SnoptSolver>;
      *iter++ = &kStaticSolver<IpoptSolver>;
      *iter++ = &kStaticSolver<NloptSolver>;
      return result;
    }
    case ProgramType::kLCP: {
      // Preferred solver.
      *iter++ = &kStaticSolver<MobyLCPSolver<double>>;
      // Dispreferred solver (generic nonlinear solver).
      *iter++ = &kStaticSolver<SnoptSolver>;
      if (!conservative) {
        // TODO(hongkai.dai): enable IPOPT and NLOPT for linear complementarity
        // constraints.
	*iter++ = &kStaticSolver<IpoptSolver>;
	*iter++ = &kStaticSolver<NloptSolver>;
      }
      return result;
    }
    case ProgramType::kUnknown: {
      if (!conservative) {
        // The order of solvers listed here is an approximation of a total
        // order, drawn from all of the partial orders given throughout the
        // other case statements shown above.
        *iter++ = &kStaticSolver<LinearSystemSolver>;
	*iter++ = &kStaticSolver<EqualityConstrainedQPSolver>;
	*iter++ = &kStaticSolver<MosekSolver>;
	*iter++ = &kStaticSolver<GurobiSolver>;
	*iter++ = &kStaticSolver<ClarabelSolver>;
	*iter++ = &kStaticSolver<OsqpSolver>;
	*iter++ = &kStaticSolver<ClpSolver>;
	*iter++ = &kStaticSolver<MobyLCPSolver<double>>;
	*iter++ = &kStaticSolver<SnoptSolver>;
	*iter++ = &kStaticSolver<IpoptSolver>;
	*iter++ = &kStaticSolver<NloptSolver>;
	*iter++ = &kStaticSolver<CsdpSolver>;
	*iter++ = &kStaticSolver<ScsSolver>;
      }
      return result;
    }
  }
  DRAKE_UNREACHABLE();
}

}  // namespace

// This function is performance sensitive, so we want to use InlinedVector and
// constexpr code to avoid heap memory allocation.
SolverId ChooseBestSolver(const MathematicalProgram& prog) {
  const ProgramType program_type = GetProgramType(prog);
  auto solvers = GetSolversFor(program_type, /* conservative= */ false);
  std::string_view additional_error_message;
  switch (program_type) {
    case ProgramType::kLP:
    case ProgramType::kQP:
    case ProgramType::kSOCP:
    case ProgramType::kSDP:
    case ProgramType::kGP:
    case ProgramType::kCGP:
    case ProgramType::kQuadraticCostConicConstraint:
    case ProgramType::kNLP:
    case ProgramType::kLCP:
    case ProgramType::kUnknown: {
      break;
    }
    case ProgramType::kMILP:
    case ProgramType::kMIQP:
    case ProgramType::kMISOCP: {
      additional_error_message =
          ", please manually instantiate MixedIntegerBranchAndBound and solve "
          "the problem if the problem size is small, typically with less than "
          "a dozen of binary variables.";
      break;
    }
    case ProgramType::kMISDP: {
      additional_error_message =
          "ChooseBestSolver():MISDP problems are not well-supported yet. You "
          "can try Drake's implementation MixedIntegerBranchAndBound for small "
          "sized MISDPs.";
      break;
    }
  }
  for (const StaticSolverInterface* solver : solvers) {
    if (solver == nullptr) {
      break;
    }
    if (solver->is_available() && solver->is_enabled() &&
	solver->AreProgramAttributesSatisfied(prog)) {
      return solver->id();
    }
  }
  throw std::invalid_argument(
      fmt::format("There is no available solver for the optimization program{}",
                  additional_error_message));
}

const std::set<SolverId>& GetKnownSolvers() {
  static const never_destroyed<std::set<SolverId>> result{[]() {
    std::set<SolverId> prototype;
    for (const auto* solver : kKnownSolvers) {
      prototype.insert(solver->id());
    }
    return prototype;
  }()};
  return result.access();
}

std::unique_ptr<SolverInterface> MakeSolver(const SolverId& id) {
  const auto& map = GetKnownSolversMap();
  auto iter = map.find(id);
  if (iter != map.end()) {
    const StaticSolverInterface& solver = *(iter->second);
    return solver.Make();
  }
  throw std::invalid_argument("MakeSolver: no matching solver " + id.name());
}

std::unique_ptr<SolverInterface> MakeFirstAvailableSolver(
    const std::vector<SolverId>& solver_ids) {
  const auto& map = GetKnownSolversMap();
  for (const auto& solver_id : solver_ids) {
    auto iter = map.find(solver_id);
    if (iter != map.end()) {
      const StaticSolverInterface& solver = *(iter->second);
      if (solver.is_available() && solver.is_enabled()) {
        return solver.Make();
      }
    }
  }
  std::string solver_names = "";
  for (const auto& solver_id : solver_ids) {
    solver_names.append(solver_id.name() + " ");
  }
  throw std::runtime_error("MakeFirstAvailableSolver(): none of the solvers " +
                           solver_names + "is available and enabled.");
}

std::vector<SolverId> GetAvailableSolvers(ProgramType prog_type) {
  std::vector<SolverId> result;
  auto solvers = GetSolversFor(prog_type, /* conservative= */ true);
  for (const StaticSolverInterface* solver : solvers) {
    if (solver == nullptr) {
      break;
    }
    if (solver->is_available() && solver->is_enabled()) {
      result.push_back(solver->id());
    }
  }
  return result;
}

}  // namespace solvers
}  // namespace drake
