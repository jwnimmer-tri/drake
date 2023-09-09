/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include): Our header file is included by polynomial.h.
#include "drake/common/symbolic/polynomial.h"
/* clang-format on */

#include <type_traits>

namespace drake {
namespace symbolic {
namespace internal {

// We have a very particular class layout. Check that it's still okay.
static_assert(sizeof(MonomialVariableToDegreeMap) <= 32);
static_assert(std::is_standard_layout_v<MonomialVariableToDegreeMap>);

MonomialVariableToDegreeMap::MonomialVariableToDegreeMap(
    const std::map<Variable, int>& powers) {
  std::vector<VariableDegreePair> new_powers;
  new_powers.reserve(powers.size());
  int new_total_degree = 0;
  for (const auto& [var, exponent] : powers) {
    DRAKE_THROW_UNLESS(exponent >= 0);
    if (exponent == 0) {
      continue;
    }
    new_powers.emplace_back(var, exponent);
    new_total_degree += exponent;
  }
  if (new_powers.empty()) {
    return;
  }
  if (new_powers.size() == 1) {
    new (&single_pair_) VariableDegreePair(std::move(new_powers.front()));
  } else {
    new (&multivariate_) Multivariate();
    multivariate_.total_degree = new_total_degree;
    multivariate_.sorted_pairs = std::move(new_powers);
  }
}

int MonomialVariableToDegreeMap::Increment(const Variable& var) {
  (void)(var);
  DRAKE_UNREACHABLE();
}

int MonomialVariableToDegreeMap::Decrement(const Variable& var) {
  (void)(var);
  DRAKE_UNREACHABLE();
}

void MonomialVariableToDegreeMap::MultiplyInPlace(
    const MonomialVariableToDegreeMap& other) {
  // Dispose of any no-ops first.
  if (other.total_degree() == 0) {
    return;
  }
  if (total_degree() == 0) {
    *this = other;
    return;
  }

  // Special case when both are the same univariate.
  if (!is_multivariate() && !other.is_multivariate()) {
    const Variable& this_var = single_pair_.first;
    const Variable& other_var = other.single_pair_.first;
    if (this_var == other_var) {
      int& this_exponent = single_pair_.second;
      const int other_exponent = other.single_pair_.second;
      this_exponent += other_exponent;
      return;
    }
  }

  // The result will be multivariate. Compute the degree before making changes.
  const int new_total_degree = total_degree() + other.total_degree();

  // To preserve our invariants as we iterate, we'll prepare the result in a
  // temporary and move it at the very end. Start by moving our current value
  // into a temporary.
  std::vector<VariableDegreePair> this_sorted_pairs;
  if (is_multivariate()) {
    this_sorted_pairs = std::move(multivariate_.sorted_pairs);
  } else {
    this_sorted_pairs.push_back(single_pair_);
  }
  clear();

  // Merge sort `this_sorted_pairs` and `other` into `new_sorted_pairs`.
  std::vector<VariableDegreePair> new_sorted_pairs;
  new_sorted_pairs.reserve(this_sorted_pairs.size() + other.size());
  auto this_iter = this_sorted_pairs.begin();
  auto other_iter = other.cbegin();
  while (this_iter != this_sorted_pairs.end() && other_iter != other.cend()) {
    const Variable& this_var = this_iter->first;
    const Variable& other_var = other_iter->first;
    if (this_var < other_var) {
      new_sorted_pairs.push_back(std::move(*this_iter));
    } else if (this_var == other_var) {
      const int this_exponent = this_iter->second;
      const int other_exponent = other_iter->second;
      new_sorted_pairs.emplace_back(this_var, this_exponent + other_exponent);
    } else {
      new_sorted_pairs.push_back(*other_iter);
    }
  }
  for (; this_iter != this_sorted_pairs.end(); ++this_iter) {
    new_sorted_pairs.push_back(std::move(*this_iter));
  }
  for (; other_iter != other.cend(); ++other_iter) {
    new_sorted_pairs.push_back(*other_iter);
  }

  // Commit the result.
  single_pair_.~VariableDegreePair();
  new (&multivariate_) Multivariate();
  multivariate_.total_degree = new_total_degree;
  multivariate_.sorted_pairs = std::move(new_sorted_pairs);
}

void MonomialVariableToDegreeMap::PowInPlace(int exponent) {
  (void)(exponent);
  DRAKE_UNREACHABLE();
}

void MonomialVariableToDegreeMap::clear() {
  if (is_multivariate()) {
    multivariate_.~Multivariate();
    new (&single_pair_) VariableDegreePair(Variable{}, 0);
  } else {
    single_pair_ = {};
  }
}

}  // namespace internal
}  // namespace symbolic
}  // namespace drake
