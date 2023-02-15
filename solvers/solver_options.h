#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_options_range.h"

namespace drake {
namespace solvers {

/** SolverOptions stores configuration options for multiple solvers in a table
of (solver_id, option_name, option_value) tuples. Values can be integers,
floating-point numbers, or strings. For a given (solver_id, option_name) pair,
at most one option_value is stored.

This class does not sanity check the option names nor types when they are
added. Instead, the solver itself will verify the options during solving.

When solving a given MathematicalProgram, only the rows associated with one
SolverId will be used, except for the CommonSolverOption settings which are
applied whenever possible. This provides a mechanism to fine-tune a
MathematicalProgram for several different solvers at once, without knowning
which one will be used to Solve().

Supported solver names/options:

"SNOPT" -- Parameter names and values as specified in SNOPT User's Guide section
7.7 "Description of the optional parameters", used as described in section 7.5
for snSet().  The SNOPT user guide can be obtained from
https://web.stanford.edu/group/SOL/guides/sndoc7.pdf

"IPOPT" -- Parameter names and values as specified in IPOPT users guide section
"Options Reference" https://coin-or.github.io/Ipopt/OPTIONS.html

"NLOPT" -- Parameter names and values are specified in
https://nlopt.readthedocs.io/en/latest/NLopt_C-plus-plus_Reference/ (in the
Stopping criteria section). Besides these parameters, the user can specify
"algorithm" using a string of the algorithm name. The complete set of algorithms
is listed in "nlopt_algorithm_to_string()" function in
github.com/stevengj/nlopt/blob/master/src/api/general.c. If you would like to
use certain algorithm, for example NLOPT_LD_SLSQP, call
`SetOption(NloptSolver::id(), NloptSolver::AlgorithmName(), "LD_SLSQP");`

"GUROBI" -- Parameter name and values as specified in Gurobi Reference Manual,
section 10.2 "Parameter Descriptions"
https://www.gurobi.com/documentation/9.5/refman/parameters.html

"SCS" -- Parameter name and values as specified in the struct SCS_SETTINGS in
SCS header file https://github.com/cvxgrp/scs/blob/master/include/scs.h Note
that the SCS code on github master might be more up-to-date than the version
used in Drake.

"MOSEK™" -- Parameter name and values as specified in Mosek Reference
https://docs.mosek.com/9.3/capi/parameters.html

"OSQP" -- Parameter name and values as specified in OSQP Reference
https://osqp.org/docs/interfaces/solver_settings.html#solver-settings */
class SolverOptions {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverOptions)

  SolverOptions() = default;

  /** A variant that can hold a copy of any specific option value.
  @note In the future, we might re-order or add more allowed types without any
  deprecation period, so be sure to use std::visit or std::get<T> to retrieve
  the variant's value in a future-proof way. */
  using OptionValue = std::variant<int, double, std::string>;

  /** A variant that can hold a view of any specific option value, or monostate
  to indicate that the option is unset. This view becomes invalid as soon as
  any non-const function is called on the SolverOptions object it came from.
  @note In the future, we might re-order or add more allowed types without any
  deprecation period, so be sure to use std::visit or std::get<T> to retrieve
  the variant's value in a future-proof way. */
  using OptionValueView =
      std::variant<std::monostate, int, double, std::string_view>;

  /** @name Setting an option
  Options are referred to either by a (solver_id, name) key pair or else a
  CommonSolverOption key. Values can be integers, floating-point numbers, or
  strings. It is not an error to set same key is set multiple times; the new
  value overtakes the old value (i.e., "last one wins"). */
  //@{
  void SetOption(const SolverId& id, std::string_view name, int value);
  void SetOption(const SolverId& id, std::string_view name, double value);
  void SetOption(const SolverId& id, std::string_view name,
                 std::string_view value);
  void SetOption(CommonSolverOption name, int value);
  void SetOption(CommonSolverOption name, std::string_view value);
  //@}

  /** @name Getting a common option */
  //@{

  /** Returns the kPrintFileName value, or "" none was set. */
  std::string_view get_print_file_name() const;

  /** Returns the kPrintToConsole value, or false if none was set. */
  bool get_print_to_console() const;

  //@}

  /** @name Getting a solver-specific option
  @tparam T can be int, double, or std::string_view.
  @note Any string_view returned here always points to a nil-terminated
  character array, so calling data() on it is equivalent to c_str() on a
  std::string; however, the nil byte is _not_ included in the length().
  */
  //@{

  /** Returns the option for the given `(id, name)`, or monostate if not set. */
  OptionValueView GetOption(const SolverId& id, std::string_view name) const;

  /** Returns the T-typed option for the given `(id, name)`, or nullopt if the
  option is not set or is set to a type other than T. */

  template <typename T>
  std::optional<T> GetOption(const SolverId& id, std::string_view name) const;

  /** Returns a range for all T-typed options for the given `id`. */
  template <typename T>
  SolverOptionsRange<T> GetRange(const SolverId& id) const;

  //@}

  /** @name Bulk operations */
  //@{

  /** Returns true iff this contains no options at all. */
  bool empty() const { return table_.empty(); }

  /** Copies all options from `other` for the given `id` into `this`. If the
  option was already set in `this`, the value in `other` takes precedence. */
  void Update(const SolverOptions& other, const SolverId& id);

  /** Copies all options into a map-of-maps. This is intended for debugging. */
  std::map<SolverId, std::map<std::string, OptionValue>> CopyToMap() const;

  /** Dumps all options into a string. Strings in the output (solver names,
  option names, and string-typed option values) will be quoted using the given
  character. */
  std::string DumpToString(std::string_view quote = {}) const;

  //@}

 private:
  template <typename>
  friend class SolverOptionsIterator;

  /* Each row of the table has a union that's discriminated using this enum.
  A tombstone is used for options that have been overwritten. */
  enum Kind : int8_t { kTombstone, kInt, kDouble, kString };

  /* Looks up the Kind tag for a given T-typed option. */
  template <typename T>
  static constexpr Kind GetKindFor() {
    if constexpr (std::is_same_v<T, int>) {
      return SolverOptions::kInt;
    }
    if constexpr (std::is_same_v<T, double>) {
      return SolverOptions::kDouble;
    }
    if constexpr (std::is_same_v<T, std::string_view>) {
      return SolverOptions::kString;
    }
  }

  /* We store all string data in a single arena, `cstring_arena_`. This struct
  denotes one string within that table. All strings are stored with a nil byte
  at the end, to be compatible with solvers implemented in C. */
  struct Str {
    /* The index into cstring_arena_ of the first character. */
    int offset{0};
    /* The string length, not including the nil byte at the end. */
    int length{0};
  };

  /* The option table is stored as rows of this type: solver_id, name, value. */
  struct Row {
    /* The solver_id for this row, we use a pseudo-SolverId when storing a
    CommonSolverOption value. */
    int solver_id{0};
    /* The option name for this row. */
    Str name;
    /* A skiplist offset into `table_` indicating how far back to go to find
    another option for the same solver_id. Zero means this is the first option
    for our solver_id. */
    int16_t skip_back{0};
    /* The discriminator for the `value` union immediately below. */
    Kind kind{kInt};
    /* The option value for this row, per the `kind` immediately above. */
    union {
      int int_value;
      double double_value;
      Str string_value;
    } value{};
  };

  /* The options are LIFO ("last in first out"). Here we keep track of the most
  recently added option for a given SolverId, and whether the tombstones are up
  to date (i.e., whether of not we've removed the earlier values for options
  that were set multiple times). */
  struct Begin {
    SolverId id;
    int table_index{0};
    mutable bool dirty{true};
  };

  /* Appends the given data to the cstring arena and returns a view to it. */
  Str AddStr(std::string_view data);

  /* Returns a view of the given Str from the arena. The result always points to
  a nil-terminated character array, so calling data() on it is equivalent to
  c_str() on a std::string; however, the nil byte is _not_ included in the
  length(). */
  std::string_view to_string_view(const Str& s) const;

  /* We are going to append an option for the given `id` onto `table_`, so we
  need to update our skip list: changes our `begins_` index to point to the next
  table row to be added as the most recently added option for that `id`, and
  returns the `skip_back` for how many rows prior the next most recently option
  for this `id` lives in the `table_`. */
  int16_t UpdateBeginsAndReturnSkipBack(const SolverId& id);

  /* Returns Begin entryfor the given `id`, or null if there are no options for
  this `id`. */
  const Begin* FindBegin(const SolverId& id) const;

  // The table of options, i.e., (id, name, value).
  std::vector<Row> table_;

  // An index to the most recently added option for a given solver id.
  std::vector<Begin> begins_;

  // The arena for all of our string data (option names and string-typed option
  // values).
  std::vector<char> cstring_arena_;
};

DRAKE_DEPRECATED(
    "2023-06-01",
    "Use fmt or spdlog for logging, not to_string. "
    "See https://github.com/RobotLocomotion/drake/issues/17742 for details.")
inline std::string to_string(const SolverOptions& x) {
  return fmt::to_string(x);
}

// TODO(jwnimmer-tri) On 2023-06-01 also remove the <ostream> include above.
DRAKE_DEPRECATED(
    "2023-06-01",
    "Use fmt or spdlog for logging, not operator<<. "
    "See https://github.com/RobotLocomotion/drake/issues/17742 for details.")
inline std::ostream& operator<<(std::ostream& os, const SolverOptions& x) {
  os << fmt::to_string(x);
  return os;
}

namespace internal {
/* Returns a placeholder SolverId used to get and set a CommonSolverOption in
the SolverOptions table. */
const SolverId& GetCommonOptionPseudoSolverId();
}  // namespace internal

}  // namespace solvers
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::solvers, SolverOptions, x, x.DumpToString())
