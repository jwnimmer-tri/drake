#pragma once

#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
/**
 * Stores options for multiple solvers.  This interface does not
 * do any verification of solver parameters. It does not even verify that
 * the specified solver exists.  Use this only when you have
 * particular knowledge of what solver is being invoked, and exactly
 * what tuning is required.
 *
 * Supported solver names/options:
 *
 * "SNOPT" -- Parameter names and values as specified in SNOPT
 * User's Guide section 7.7 "Description of the optional parameters",
 * used as described in section 7.5 for snSet().
 * The SNOPT user guide can be obtained from
 * https://web.stanford.edu/group/SOL/guides/sndoc7.pdf
 *
 * "IPOPT" -- Parameter names and values as specified in IPOPT users
 * guide section "Options Reference"
 * http://www.coin-or.org/Ipopt/documentation/node40.html
 *
 * "GUROBI" -- Parameter name and values as specified in Gurobi Reference
 * Manual, section 10.2 "Parameter Descriptions"
 * https://www.gurobi.com/documentation/9.0/refman/parameters.html
 *
 * "SCS" -- Parameter name and values as specified in the struct SCS_SETTINGS in
 * SCS header file https://github.com/cvxgrp/scs/blob/master/include/scs.h
 * Note that the SCS code on github master might be more up-to-date than the
 * version used in Drake.
 *
 * "MOSEK" -- Parameter name and values as specified in Mosek Reference
 * https://docs.mosek.com/9.2/capi/parameters.html
 *
 * "OSQP" -- Parameter name and values as specified in OSQP Reference
 * https://osqp.org/docs/interfaces/solver_settings.html#solver-settings
 *
 * "dReal" -- Parameter name and values as specified in dReal Reference
 * https://github.com/dreal/dreal4/blob/master/README.md#command-line-options.
 * Note that Drake only supports a subset of the options listed in the
 * reference. @see DrealSolver for the subset of these options supported by the
 * solver interface.
 */
class SolverOptions {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverOptions)

  SolverOptions() = default;

  // XXX group into setters, for users

  /** The values stored in SolverOptions can be double, int, or string. In the
  future, we might re-order or add more allowed types without any deprecation
  period, so be sure to use std::visit or std::get<T> to retrieve the variant's
  value in a future-proof way. */
  using OptionValue = std::variant<double, int, std::string>;

  /** Sets a common option for all solvers supporting that option (for example,
  printing the progress in each iteration). If the solver doesn't support the
  option, then the option is silently ignored. */
  void SetOption(CommonSolverOption key, OptionValue value);

  /** Sets a solver option for a specific solver. */
  void SetOption(const SolverId& solver_id, std::string solver_option,
                 OptionValue option_value);

  // XXX group into getters, for end solver implementations.

  /** Returns the kPrintFileName set via CommonSolverOption, or else an empty
  string if the option has not been set. */
  std::string get_print_file_name() const;

  /** Returns the kPrintToConsole set via CommonSolverOption, or else false if
  the option has not been set. */
  bool get_print_to_console() const;

  /** Returns ... */
  const std::map<std::string, OptionValue>& GetOptions(
      const SolverId& solver_id) const;

  // XXX group into infra, for SolverBase etc.

  /**
   * Merges the other solver options into this. If `other` and `this` option
   * both define the same option for the same solver, we ignore then one from
   * `other` and keep the one from `this`.
   */
  void Merge(const SolverOptions& other);

  DRAKE_DEPRECATED("2021-12-01", "")
  const std::unordered_map<std::string, double>& GetOptionsDouble(
      const SolverId& solver_id) const;

  DRAKE_DEPRECATED("2021-12-01", "")
  const std::unordered_map<std::string, int>& GetOptionsInt(
      const SolverId& solver_id) const;

  DRAKE_DEPRECATED("2021-12-01", "")
  const std::unordered_map<std::string, std::string>& GetOptionsStr(
      const SolverId& solver_id) const;

  template <typename T>
  DRAKE_DEPRECATED("2021-12-01", "")
  const std::unordered_map<std::string, T>& GetOptions(
      const SolverId& solver_id) const {
    if constexpr (std::is_same_v<T, double>) {
      return GetOptionsDouble(solver_id);
    } else if constexpr (std::is_same_v<T, int>) {
      return GetOptionsInt(solver_id);
    } else if constexpr (std::is_same_v<T, std::string>) {
      return GetOptionsStr(solver_id);
    }
    DRAKE_UNREACHABLE();
  }

  DRAKE_DEPRECATED("2021-12-01", "")
  const std::unordered_map<CommonSolverOption, OptionValue>&
  common_solver_options() const { return common_; }

  DRAKE_DEPRECATED("2021-12-01", "")
  std::unordered_set<SolverId> GetSolverIds() const;

  DRAKE_DEPRECATED("2021-12-01", "")
  bool operator==(const SolverOptions& other) const;

  DRAKE_DEPRECATED("2021-12-01", "")
  bool operator!=(const SolverOptions& other) const;

  DRAKE_DEPRECATED("2021-12-01", "")
  void CheckOptionKeysForSolver(
      const SolverId& solver_id,
      const std::unordered_set<std::string>& allowable_double_keys,
      const std::unordered_set<std::string>& allowable_int_keys,
      const std::unordered_set<std::string>& allowable_str_keys) const;

 private:
  std::unordered_map<CommonSolverOption, OptionValue> common_;
  std::unordered_map<SolverId, std::map<std::string, OptionValue>> specific_;

  // On 2021-12-01 when we remove the deprecated getters, we should also remove
  // these member fields.
  std::unordered_map<SolverId, std::unordered_map<std::string, double>>
      solver_options_double_{};
  std::unordered_map<SolverId, std::unordered_map<std::string, int>>
      solver_options_int_{};
  std::unordered_map<SolverId, std::unordered_map<std::string, std::string>>
      solver_options_str_{};
};

std::string to_string(const SolverOptions&);
std::ostream& operator<<(std::ostream&, const SolverOptions&);

}  // namespace solvers
}  // namespace drake
