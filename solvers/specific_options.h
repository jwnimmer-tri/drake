#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>

#include "drake/common/drake_assert.h"
#include "drake/common/name_value.h"
#include "drake/common/string_unordered_map.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace solvers {
namespace internal {

/* Helper class that propagates options for a specific solver from Drake's
generic SolverOptions struct into the specific solver's options API.

This class is intended a short-lived helper. It aliases an immutable list of
overall SolverOptions, and provides convenient tools to map those options into
whatever back-end is necessary. Two different mechanisms are offered, depending
on whether the solver's option names are static or dynamic: CopyToCallbacks or
CopyToSerializableStruct.

(Method 1) CopyToCallbacks: This method is appropriate for solvers that offer an
API like "set option via string name". The caller provides several callbacks
(one for each supported data type, i.e., double, int, string) and this class
will loop over the stored options and invoke the callbacks. It is up to the
solver back-end to detect unknown or mis-typed options and throw.

(Method 2) CopyToSerializableStruct: This method is approviate for solvers that
offer a statically styped `struct MyOptions { ... }` which needs to be filled
in. The caller provides a Serialize() wrapper around that struct, and then this
class will directly write the fields and report errors for unknown names and
mismatched data types.

With both methods, there are also some ahead-of-time operations that are often
useful:

- Respell() can project CommonSolverOption values into the back-end vocabularly
  so that only the back-end specific names need to be implemented.

- Pop() can yank options that require special handling, so that they will not
  participate in the Method 1 or 2 copying / callbacks.

For examples of use, refer to all of the exisiting Drake solver wrappers. */
class SpecificOptions {
 public:
  /* Creates a converter that reads the subset of the `all_options` destined
  for the given `id`. Both options are aliased, so must outlive this object. */
  SpecificOptions(const SolverId* id, const SolverOptions* all_options);

  ~SpecificOptions();

  /* The `respell` callback will be used to respell the CommonSolverOption
  values. Any options returned will be handled as if they were solver-specific
  options, at a lower priority than any solver-specific options the user already
  provided for those names (in our constructor). It is OK to have side-effects;
  the callback will be inboked exactly once and is not retained.
  @pre This function may be called at most once (per converter object). */
  void Respell(
      const std::function<string_unordered_map<SolverOptions::OptionValue>(
          const CommonSolverOptionValues&)>& respell);

  /* ...
  @tparam T must be one of: double, int, bool, std::string. */
  template <typename Result>
  std::optional<Result> Pop(std::string_view key);

  /* Helper that converts options when the solver offers a generic key-value API
  where options are passed by string name. All options are passed to the given
  callbacks one at a time. */
  void CopyToCallbacks(
      const std::function<void(const std::string& key, double value)>&
          set_double,
      const std::function<void(const std::string& key, int value)>& set_int,
      const std::function<void(const std::string& key,
                               const std::string& value)>& set_string) const;

  /* Helper that converts options when the solver has a fixed set of option
  names at compile time (e.g., in a helper struct). The `Output` struct must
  obey Drake's Serialize() protocol, for us to interrogate the option names.
  @param [in,out] result the solver's specific options struct; any options
    not mentioned in `solver_options` will remain unchanged. */
  template <typename Result>
  void CopyToSerializableStruct(Result* result) {
    DRAKE_DEMAND(result != nullptr);
    InitializePending();
    Serialize(this, *result);
    CheckNoPending();
  }

  /* Helper function for CopyToSerializableStruct(). */
  template <typename T>
  void Visit(const NameValue<T>& x) {
    if constexpr (std::is_floating_point_v<T>) {
      CopyFloatingPointOption(x.name(), x.value());
    } else if constexpr (std::is_integral_v<T>) {
      CopyIntegralOption(x.name(), x.value());
    } else if constexpr (std::is_same_v<T, std::string>) {
      CopyStringOption(x.name(), x.value());
    } else {
      // XXX
    }
  }

 private:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpecificOptions);

  static CommonSolverOptionValues ExtractCommon(
      const SolverOptions& solver_options);

  /* ... */
  void InitializePending();

  /* If pending_keys_ is non-empty, throws an error about unhandled options. */
  void CheckNoPending() const;

  /* ... */
  const SolverOptions::OptionValue* PrepareToCopy(const char* name);

  /* Sets `output` to the value specified for the given `name` from `options_`,
  and removes the name from `pending_keys_`. If the name is not in options,
  does nothing.
  @tparam T must be one of: float, double. */
  template <typename T>
  void CopyFloatingPointOption(const char* name, T* output);

  /* Sets `output` to the value specified for the given `name` from `options_`,
  and removes the name from `pending_keys_`. If the name is not in options,
  does nothing.
  @tparam T must be one of: int, bool, uint32_t. */
  template <typename T>
  void CopyIntegralOption(const char* name, T* output);

  /* Sets `output` to the value specified for the given `name` from `options_`,
  and removes the name from `pending_keys_`. If the name is not in options,
  does nothing. */
  void CopyStringOption(const char* name, std::string* output);

  // The full options for all solvers (as passed to our constructor).
  const SolverOptions* const all_options_;

  // The name of the selected solver (per the ID passed to our constructor).
  const std::string solver_name_;

  // The options for the selected solver (an alias into `all_options_`).
  // When there are no specific options this will be nullptr (not empty).
  const string_unordered_map<SolverOptions::OptionValue>* specific_options_{};

  // The result of the Respell() callback (or empty, if never called).
  string_unordered_map<SolverOptions::OptionValue> respelled_;

  // The keys of `options_` that are yet to be processed. This starts out as the
  // the full set and gradually shinks as we copy out the options.
  std::unordered_set<std::string_view> pending_keys_;
};

}  // namespace internal
}  // namespace solvers
}  // namespace drake
