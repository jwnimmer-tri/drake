#include "drake/solvers/osqp_solver.h"

#include <iostream>
#include <vector>

#include <osqp.h>

#include "drake/common/text_logging.h"
#include "drake/common/scope_exit.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/mathematical_program.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtautological-compare"
static_assert(c_malloc == malloc,
    "Our code below assumes that malloc is not redirected");
static_assert(c_free == free,
    "Our code below assumes that free is not redirected");
#pragma GCC diagnostic pop

namespace drake {
namespace solvers {
namespace {
void ParseQuadraticCosts(const MathematicalProgram& prog,
                         Eigen::SparseMatrix<c_float>* P,
                         std::vector<c_float>* q, double* constant_cost_term) {
  DRAKE_ASSERT(static_cast<int>(q->size()) == prog.num_vars());

  // Loop through each quadratic costs in prog, and compute the Hessian matrix
  // P, the linear cost q, and the constant cost term.
  std::vector<Eigen::Triplet<c_float>> P_triplets;
  for (const auto& quadratic_cost : prog.quadratic_costs()) {
    const VectorXDecisionVariable& x = quadratic_cost.variables();
    // x_indices are the indices of the variables x (the variables bound with
    // this quadratic cost) in the program decision variables.
    const std::vector<int> x_indices = prog.FindDecisionVariableIndices(x);

    // Add quadratic_cost.Q to the Hessian P.
    const std::vector<Eigen::Triplet<double>> Qi_triplets =
        math::SparseMatrixToTriplets(quadratic_cost.evaluator()->Q());
    P_triplets.reserve(P_triplets.size() + Qi_triplets.size());
    for (int i = 0; i < static_cast<int>(Qi_triplets.size()); ++i) {
      // Unpack the field of the triplet (for clarity below).
      const int row = x_indices[Qi_triplets[i].row()];
      const int col = x_indices[Qi_triplets[i].col()];
      const double value = Qi_triplets[i].value();
      // Since OSQP 0.6.0 the P matrix is required to be upper triangular, so
      // we only add upper triangular entries to P_triplets.
      if (row <= col) {
        P_triplets.emplace_back(row, col, static_cast<c_float>(value));
      }
    }

    // Add quadratic_cost.b to the linear cost term q.
    for (int i = 0; i < x.rows(); ++i) {
      q->at(x_indices[i]) += quadratic_cost.evaluator()->b()(i);
    }

    // Add quadratic_cost.c to constant term
    *constant_cost_term += quadratic_cost.evaluator()->c();
  }

  // Scale the matrix P in the cost.
  // Note that the linear term is scaled in ParseLinearCosts().
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (auto& triplet : P_triplets) {
      // Column
      const auto column = scale_map.find(triplet.col());
      if (column != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (column->second));
      }
      // Row
      const auto row = scale_map.find(triplet.row());
      if (row != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (row->second));
      }
    }
  }

  P->resize(prog.num_vars(), prog.num_vars());
  P->setFromTriplets(P_triplets.begin(), P_triplets.end());
}

void ParseLinearCosts(const MathematicalProgram& prog, std::vector<c_float>* q,
                      double* constant_cost_term) {
  // Add the linear costs to the OSQP cost.
  DRAKE_ASSERT(static_cast<int>(q->size()) == prog.num_vars());

  // Loop over the linear costs stored inside prog.
  for (const auto& linear_cost : prog.linear_costs()) {
    for (int i = 0; i < static_cast<int>(linear_cost.GetNumElements()); ++i) {
      // Append the linear cost term to q.
      if (linear_cost.evaluator()->a()(i) != 0) {
        const int x_index =
            prog.FindDecisionVariableIndex(linear_cost.variables()(i));
        q->at(x_index) += linear_cost.evaluator()->a()(i);
      }
    }
    // Add the constant cost term to constant_cost_term.
    *constant_cost_term += linear_cost.evaluator()->b();
  }

  // Scale the vector q in the cost.
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (const auto& [index, scale] : scale_map) {
      q->at(index) *= scale;
    }
  }
}

// OSQP defines its own infinity in osqp/include/glob_opts.h.
c_float ConvertInfinity(double val) {
  if (std::isinf(val)) {
    if (val > 0) {
      return OSQP_INFTY;
    }
    return -OSQP_INFTY;
  }
  return static_cast<c_float>(val);
}

// Will call this function to parse both LinearConstraint and
// LinearEqualityConstraint.
template <typename C>
void ParseLinearConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& linear_constraints,
    std::vector<Eigen::Triplet<c_float>>* A_triplets, std::vector<c_float>* l,
    std::vector<c_float>* u, int* num_A_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : linear_constraints) {
    const std::vector<int> x_indices =
        prog.FindDecisionVariableIndices(constraint.variables());
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(constraint.evaluator()->A());
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_A_rows);
    // Append constraint.A to OSQP A.
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(*num_A_rows + Ai_triplet.row(),
                               x_indices[Ai_triplet.col()],
                               static_cast<c_float>(Ai_triplet.value()));
    }
    const int num_Ai_rows = constraint.evaluator()->num_constraints();
    l->reserve(l->size() + num_Ai_rows);
    u->reserve(u->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      l->push_back(ConvertInfinity(constraint.evaluator()->lower_bound()(i)));
      u->push_back(ConvertInfinity(constraint.evaluator()->upper_bound()(i)));
    }
    *num_A_rows += num_Ai_rows;
  }
}

void ParseBoundingBoxConstraints(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<c_float>>* A_triplets, std::vector<c_float>* l,
    std::vector<c_float>* u, int* num_A_rows,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  // Loop over the linear constraints, stack them to get l, u and A.
  for (const auto& constraint : prog.bounding_box_constraints()) {
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    constraint_start_row->emplace(constraint_cast, *num_A_rows);
    // Append constraint.A to OSQP A.
    for (int i = 0; i < static_cast<int>(constraint.GetNumElements()); ++i) {
      A_triplets->emplace_back(
          *num_A_rows + i,
          prog.FindDecisionVariableIndex(constraint.variables()(i)),
          static_cast<c_float>(1));
    }
    const int num_Ai_rows = constraint.evaluator()->num_constraints();
    l->reserve(l->size() + num_Ai_rows);
    u->reserve(u->size() + num_Ai_rows);
    for (int i = 0; i < num_Ai_rows; ++i) {
      l->push_back(ConvertInfinity(constraint.evaluator()->lower_bound()(i)));
      u->push_back(ConvertInfinity(constraint.evaluator()->upper_bound()(i)));
    }
    *num_A_rows += num_Ai_rows;
  }
}

// @param[out] A
// @param[out] l are lower bounds
// @param[out] u are lower bounds
// @param[out] constraint_start_row
void ParseAllLinearConstraints(
    const MathematicalProgram& prog, Eigen::SparseMatrix<c_float>* A,
    std::vector<c_float>* l, std::vector<c_float>* u,
    std::unordered_map<Binding<Constraint>, int>* constraint_start_row) {
  DRAKE_DEMAND(A->nonZeros() == 0);
  DRAKE_DEMAND(l->empty());
  DRAKE_DEMAND(u->empty());
  DRAKE_DEMAND(constraint_start_row->empty());
  std::vector<Eigen::Triplet<c_float>> A_triplets;
  int num_A_rows = 0;
  ParseLinearConstraints(prog, prog.linear_constraints(), &A_triplets, l, u,
                         &num_A_rows, constraint_start_row);
  ParseLinearConstraints(prog, prog.linear_equality_constraints(), &A_triplets,
                         l, u, &num_A_rows, constraint_start_row);
  ParseBoundingBoxConstraints(prog, &A_triplets, l, u, &num_A_rows,
                              constraint_start_row);

  // Scale the matrix A.
  // Note that we only scale the columns of A, because the constraint has the
  // form l <= Ax <= u where the scaling of x enters the columns of A instead of
  // rows of A.
  const auto& scale_map = prog.GetVariableScaling();
  if (!scale_map.empty()) {
    for (auto& triplet : A_triplets) {
      auto column = scale_map.find(triplet.col());
      if (column != scale_map.end()) {
        triplet = Eigen::Triplet<double>(triplet.row(), triplet.col(),
                                         triplet.value() * (column->second));
      }
    }
  }

  A->resize(num_A_rows, prog.num_vars());
  A->setFromTriplets(A_triplets.begin(), A_triplets.end());
}

// Convert an Eigen::SparseMatrix to csc_matrix to be used by OSQP.
//
// @param[in,out] result points to an owned scs struct. If the pointed-to
// value is null, then this function will allocate a new struct.  If the value
// is non-null, then the existing struct will be reused if appropriately sized,
// or else free'd and then malloc'd again.
//
// @post Either way, *result will be non-null and contain a copy of the values
// in @p mat and the caller is responsible for calling csc_spfree() on it.
void EigenSparseToCSC(
    const Eigen::SparseMatrix<c_float>& mat,
    csc** result) {
  DRAKE_DEMAND(result != nullptr);

  const int non_zeros = mat.nonZeros();
  const int rows = mat.rows();
  const int cols = mat.cols();

  // A csc matrix uses the compressed column major format.
  const int num_values = non_zeros;
  const int num_inner_indices = non_zeros;
  const int num_outer_indices = cols + 1;

  // Allocate or re-allocate storage when necessary.
  const bool needs_allocation =
      (*result == nullptr) ||
      ((**result).nzmax != non_zeros) ||
      ((**result).n != cols);
  if (needs_allocation) {
    if (*result != nullptr) {
      csc_spfree(*result);
    }
    c_float* values = static_cast<c_float*>(
        malloc(num_values * sizeof(c_float)));
    c_int* inner_indices = static_cast<c_int*>(
        malloc(num_inner_indices * sizeof(c_int)));
    c_int* outer_indices = static_cast<c_int*>(
        malloc(num_outer_indices * sizeof(c_int)));
    *result = csc_matrix(rows, cols, non_zeros, values, inner_indices,
        outer_indices);
  }
  DRAKE_DEMAND((**result).nzmax == non_zeros);
  DRAKE_DEMAND((**result).m == rows);
  DRAKE_DEMAND((**result).n == cols);

  // Copy the matrix data.
  for (int i = 0; i < num_values; ++i) {
    (**result).x[i] = *(mat.valuePtr() + i);
  }
  for (int i = 0; i < num_inner_indices; ++i) {
    (**result).i[i] = static_cast<c_int>(*(mat.innerIndexPtr() + i));
  }
  for (int i = 0; i < num_outer_indices; ++i) {
    (**result).p[i] = static_cast<c_int>(*(mat.outerIndexPtr() + i));
  }
}

template <typename T1, typename T2>
void SetOsqpSolverSetting(const std::unordered_map<std::string, T1>& options,
                          const std::string& option_name,
                          T2* osqp_setting_field) {
  const auto it = options.find(option_name);
  if (it != options.end()) {
    *osqp_setting_field = it->second;
  }
}

void SetOsqpSolverSettings(const SolverOptions& solver_options,
                           OSQPSettings* settings) {
  const std::unordered_map<std::string, double>& options_double =
      solver_options.GetOptionsDouble(OsqpSolver::id());
  const std::unordered_map<std::string, int>& options_int =
      solver_options.GetOptionsInt(OsqpSolver::id());
  SetOsqpSolverSetting(options_double, "rho", &(settings->rho));
  SetOsqpSolverSetting(options_double, "sigma", &(settings->sigma));
  SetOsqpSolverSetting(options_int, "max_iter", &(settings->max_iter));
  SetOsqpSolverSetting(options_double, "eps_abs", &(settings->eps_abs));
  SetOsqpSolverSetting(options_double, "eps_rel", &(settings->eps_rel));
  SetOsqpSolverSetting(options_double, "eps_prim_inf",
                       &(settings->eps_prim_inf));
  SetOsqpSolverSetting(options_double, "eps_dual_inf",
                       &(settings->eps_dual_inf));
  SetOsqpSolverSetting(options_double, "alpha", &(settings->alpha));
  SetOsqpSolverSetting(options_double, "delta", &(settings->delta));
  SetOsqpSolverSetting(options_int, "polish", &(settings->polish));
  SetOsqpSolverSetting(options_int, "polish_refine_iter",
                       &(settings->polish_refine_iter));
  SetOsqpSolverSetting(options_int, "verbose", &(settings->verbose));
  SetOsqpSolverSetting(options_int, "scaled_termination",
                       &(settings->scaled_termination));
  SetOsqpSolverSetting(options_int, "check_termination",
                       &(settings->check_termination));
  SetOsqpSolverSetting(options_int, "warm_start", &(settings->warm_start));
  SetOsqpSolverSetting(options_int, "scaling", &(settings->scaling));
  SetOsqpSolverSetting(options_int, "adaptive_rho", &(settings->adaptive_rho));
  SetOsqpSolverSetting(options_double, "adaptive_rho_interval",
                       &(settings->adaptive_rho_interval));
  SetOsqpSolverSetting(options_double, "adaptive_rho_tolerance",
                       &(settings->adaptive_rho_tolerance));
  SetOsqpSolverSetting(options_double, "adaptive_rho_fraction",
                       &(settings->adaptive_rho_fraction));
  SetOsqpSolverSetting(options_double, "time_limit", &(settings->time_limit));
}

template <typename C>
void SetDualSolution(
    const std::vector<Binding<C>>& constraints,
    const Eigen::VectorXd& all_dual_solution,
    const std::unordered_map<Binding<Constraint>, int>& constraint_start_row,
    MathematicalProgramResult* result) {
  for (const auto& constraint : constraints) {
    // OSQP uses the dual variable `y` as the negation of the shadow price, so
    // we need to negate `all_dual_solution` as Drake interprets dual solution
    // as the shadow price.
    const Binding<Constraint> constraint_cast =
        internal::BindingDynamicCast<Constraint>(constraint);
    result->set_dual_solution(
        constraint,
        -all_dual_solution.segment(constraint_start_row.at(constraint_cast),
                                   constraint.evaluator()->num_constraints()));
  }
}

struct Workspace {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Workspace)

  Workspace() = default;

  ~Workspace() {
    csc_spfree(P);
    csc_spfree(A);
  }

  std::unique_ptr<Workspace> Clone() const {
    // We use reset_on_copy semantics, i.e., copy-construction is a no-op.
    return std::make_unique<Workspace>();
  }

  void Clear() {
    P_sparse.setZero();
    // XXX P.clear
    q.clear();
    A_sparse.setZero();
    // XXX A.clear
    A_lower.clear();
    A_upper.clear();
  }

  // The consolidated Hessian matrix of quadratic costs.
  Eigen::SparseMatrix<c_float> P_sparse;
  csc* P = nullptr;

  // The consolidated coefficients of linear costs.
  std::vector<c_float> q;

  // The consolidated coefficients of linear constraints.
  Eigen::SparseMatrix<c_float> A_sparse;
  csc* A = nullptr;

  // The upper and lower bounds A_sparse;
  std::vector<c_float> A_lower;
  std::vector<c_float> A_upper;
};

}  // namespace

bool OsqpSolver::is_available() { return true; }

void OsqpSolver::DoSolve(
    const MathematicalProgram& prog,
    const Eigen::VectorXd& initial_guess,
    const SolverOptions& merged_options,
    MathematicalProgramResult* result) const {
  OsqpSolverDetails& solver_details =
      result->SetSolverDetailsType<OsqpSolverDetails>();
  if (solver_details.workspace == nullptr) {
    solver_details.workspace = std::make_unique<Value<Workspace>>();
  }
  Workspace& workspace =
      solver_details.workspace->get_mutable_value<Workspace>();
  workspace.Clear();

  // TODO(hongkai.dai): OSQP uses initial guess to warm start.
  unused(initial_guess);

  // OSQP solves a convex quadratic programming problem
  // min 0.5 xᵀPx + qᵀx
  // s.t l ≤ Ax ≤ u
  // OSQP is written in C, so this function will be in C style.

  // Get the cost for the QP.
  workspace.q.assign(prog.num_vars(), c_float{0});
  double constant_cost_term{0};
  ParseQuadraticCosts(prog, &workspace.P_sparse, &workspace.q,
      &constant_cost_term);
  ParseLinearCosts(prog, &workspace.q, &constant_cost_term);

  // linear_constraint_start_row[binding] stores the starting row index in A
  // corresponding to the linear constraint `binding`.
  // XXX Remove this allocation, somehow.
  std::unordered_map<Binding<Constraint>, int> constraint_start_row;

  // Parse the linear constraints.
  ParseAllLinearConstraints(prog, &workspace.A_sparse, &workspace.A_lower,
      &workspace.A_upper, &constraint_start_row);

  // Now pass the constraint and cost to osqp data.
  ::OSQPData osqp_data{};
  osqp_data.n = prog.num_vars();
  osqp_data.m = workspace.A_sparse.rows();
  EigenSparseToCSC(workspace.P_sparse, &workspace.P);
  osqp_data.P = workspace.P;
  osqp_data.q = workspace.q.data();
  EigenSparseToCSC(workspace.A_sparse, &workspace.A);
  osqp_data.A = workspace.A;
  osqp_data.l = workspace.A_lower.data();
  osqp_data.u = workspace.A_upper.data();

  // Consolidate the OSQP defaults + our defaults + solve options.
  // Default polish to true, to get an accurate solution.
  OSQPSettings osqp_settings{};
  osqp_set_default_settings(&osqp_settings);
  osqp_settings.polish = 1;
  osqp_settings.verbose = 0;
  SetOsqpSolverSettings(merged_options, &osqp_settings);

  // Setup workspace.
  OSQPWorkspace* work = nullptr;
  ScopeExit guard([work]() { osqp_cleanup(work); });
  if (osqp_setup(&work, &osqp_data, &osqp_settings) != 0) {
    result->set_solution_result(SolutionResult::kInvalidInput);
    return;
  }
  DRAKE_THROW_UNLESS(work != nullptr);

  // Solve problem.
  if (osqp_solve(work) != 0) {
    result->set_solution_result(SolutionResult::kInvalidInput);
    return;
  }
  DRAKE_THROW_UNLESS(work->info != nullptr);

  // Copy some details into the result.
  solver_details.iter = work->info->iter;
  solver_details.status_val = work->info->status_val;
  solver_details.primal_res = work->info->pri_res;
  solver_details.dual_res = work->info->dua_res;
  solver_details.setup_time = work->info->setup_time;
  solver_details.solve_time = work->info->solve_time;
  solver_details.polish_time = work->info->polish_time;
  solver_details.run_time = work->info->run_time;

  // Convert the OSQP result into a MathematicalProgramResult.
  std::optional<SolutionResult> solution_result;
  switch (work->info->status_val) {
    case OSQP_SOLVED:
    case OSQP_SOLVED_INACCURATE: {
      const Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> osqp_sol(
          work->solution->x, prog.num_vars());

      // Scale solution back if `scale_map` is not empty.
      const auto& scale_map = prog.GetVariableScaling();
      if (!scale_map.empty()) {
        drake::VectorX<double> scaled_sol = osqp_sol.cast<double>();
        for (const auto& [index, scale] : scale_map) {
          scaled_sol(index) *= scale;
        }
        result->set_x_val(scaled_sol);
      } else {
        result->set_x_val(osqp_sol.cast<double>());
      }

      result->set_optimal_cost(work->info->obj_val + constant_cost_term);
      solver_details.y =
          Eigen::Map<Eigen::VectorXd>(work->solution->y, work->data->m);
      solution_result = SolutionResult::kSolutionFound;
      SetDualSolution(prog.linear_constraints(), solver_details.y,
                      constraint_start_row, result);
      SetDualSolution(prog.linear_equality_constraints(), solver_details.y,
                      constraint_start_row, result);
      SetDualSolution(prog.bounding_box_constraints(), solver_details.y,
                      constraint_start_row, result);

      break;
    }
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE: {
      solution_result = SolutionResult::kInfeasibleConstraints;
      result->set_optimal_cost(MathematicalProgram::kGlobalInfeasibleCost);
      break;
    }
    case OSQP_DUAL_INFEASIBLE:
    case OSQP_DUAL_INFEASIBLE_INACCURATE: {
      solution_result = SolutionResult::kDualInfeasible;
      break;
    }
    case OSQP_MAX_ITER_REACHED: {
      solution_result = SolutionResult::kIterationLimit;
      break;
    }
    default: {
      solution_result = SolutionResult::kUnknownError;
      break;
    }
  }
  result->set_solution_result(solution_result.value());
}

}  // namespace solvers
}  // namespace drake
