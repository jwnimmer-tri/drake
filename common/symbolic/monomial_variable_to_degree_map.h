#pragma once

#include <map>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic/expression.h"

// As with monomial.h, this file should never be used independently.
#ifndef DRAKE_COMMON_SYMBOLIC_POLYNOMIAL_H
// NOLINTNEXTLINE(whitespace/line_length)
#error Do not directly include this file. Use "drake/common/symbolic/polynomial.h".
#endif

namespace drake {
namespace symbolic {
namespace internal {

/* A container type that approximates the std::map<Variable, int> API, but with
a more efficient implementaiton. This is used by Monomial for its powers map.
Users should refer to the documentation of Monomial::get_powers() for details.
*/
class MonomialVariableToDegreeMap final {
 public:
  // DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MonomialVariableToDegreeMap)

  /* (Internal use only) The tag type for our variant-like `union`. All members
  of the `union` have this as their first member field. Unfortunately, we can't
  use std::variant<> because it can't pack the 1-byte variant `index` alongside
  the 4-byte `degree` field. To fit in 32 bytes we need custom code. */
  struct Tag {
    // When true, the union stores a `struct Multivariate` as `multivariate_`.
    // When false, the union stores a `VariableDegreePair` as `single_pair_`.
    bool is_multivariate{false};
  };

  /* (Internal use only) The value_type for our pseudo-map. The fields are named
  like std::pair, but the storage layout is carefully customized. */
  struct VariableDegreePair {
    VariableDegreePair() = default;
    VariableDegreePair(Variable v, int degree)
        : second(degree), first(std::move(v)) {}

    /* Support for structured bindings. */
    template <size_t Index>
    decltype(auto) get() const {
      if constexpr (Index == 0) return first;
      if constexpr (Index == 1) return second;
    }
    template <size_t Index>
    friend decltype(auto) get(const VariableDegreePair& self) {
      return self.get<Index>();
    }

    /* Automatically convert to the std::map<>::value_type. */
    operator std::pair<const Variable, int>() const { return {first, second}; }

    /* Implements the @ref hash_append concept. */
    template <class HashAlgorithm>
    friend void hash_append(HashAlgorithm& hasher,
                            const VariableDegreePair& item) noexcept {
      using drake::hash_append;
      hash_append(hasher, item.first);
      hash_append(hasher, item.second);
    }

    // (Internal use only) Dummy storage so that our `union {}` trick works.
    Tag _;

    // This is the degree of the Variable named by `first`.
    int second{};

    Variable first;
  };

  // Mimic the std::map API. These typedefs are intended for users to rely on,
  // per the Monomial::get_power() documentation about std::map compatibility.
  using key_type = Variable;
  using mapped_type = int;
  using value_type = VariableDegreePair;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;
  using reference = value_type&;
  using const_reference = const value_type&;
  using pointer = value_type*;
  using const_pointer = const value_type*;
  using iterator = pointer;
  using const_iterator = const_pointer;

  /* (Internal use only) Creates an empty map. Internally, we represent this as
  the pair 𝑥⁰. */
  MonomialVariableToDegreeMap() : MonomialVariableToDegreeMap(Variable{}, 0) {}

  /* (Internal use only) Constructs the univariate monomial `pow(v, degree)`,
  unless the degree is zero in which case constructs and empty map. */
  explicit MonomialVariableToDegreeMap(const Variable& v, int degree = 1) {
    DRAKE_THROW_UNLESS(degree >= 0);
    new (&single_pair_) VariableDegreePair(degree > 0 ? v : Variable{}, degree);
  }

  /* (Internal use only) Constructs by copying the given `powers`. */
  explicit MonomialVariableToDegreeMap(const std::map<Variable, int>& powers);

  /* Copy constructor. We can't use the `= default` due to our `union`. */
  MonomialVariableToDegreeMap(const MonomialVariableToDegreeMap& other) {
    if (other.is_multivariate()) {
      new (&multivariate_) Multivariate(other.multivariate_);
    } else {
      new (&single_pair_) VariableDegreePair(other.single_pair_);
    }
  }

  /* Move constructor. We can't use the `= default` due to our `union`. */
  MonomialVariableToDegreeMap(MonomialVariableToDegreeMap&& other) {
    if (other.is_multivariate()) {
      new (&multivariate_) Multivariate(std::move(other.multivariate_));
      other.multivariate_.~Multivariate();
      new (&other.single_pair_) VariableDegreePair();
    } else {
      new (&single_pair_) VariableDegreePair(std::move(other.single_pair_));
      other.single_pair_ = {};
    }
  }

  /* Copy assignment. We can't use the `= default` due to our `union`. */
  MonomialVariableToDegreeMap& operator=(
      const MonomialVariableToDegreeMap& other) {
    if (this != &other) {
      if (is_multivariate()) {
        if (other.is_multivariate()) {
          multivariate_ = other.multivariate_;
        } else {
          multivariate_.~Multivariate();
          new (&single_pair_) VariableDegreePair(other.single_pair_);
        }
      } else {
        if (other.is_multivariate()) {
          single_pair_.~VariableDegreePair();
          new (&multivariate_) Multivariate(other.multivariate_);
        } else {
          single_pair_ = other.single_pair_;
        }
      }
    }
    return *this;
  }

  /* Move assignment. We can't use the `= default` due to our `union`. */
  MonomialVariableToDegreeMap& operator=(MonomialVariableToDegreeMap&& other) {
    if (this != &other) {
      if (is_multivariate()) {
        if (other.is_multivariate()) {
          multivariate_ = std::move(other.multivariate_);
          other.multivariate_.~Multivariate();
          new (&other.single_pair_) VariableDegreePair();
        } else {
          multivariate_.~Multivariate();
          new (&single_pair_) VariableDegreePair(std::move(other.single_pair_));
          other.single_pair_ = {};
        }
      } else {
        if (other.is_multivariate()) {
          single_pair_.~VariableDegreePair();
          new (&multivariate_) Multivariate(std::move(other.multivariate_));
          other.multivariate_.~Multivariate();
          new (&other.single_pair_) VariableDegreePair();
        } else {
          single_pair_ = std::move(other.single_pair_);
          other.single_pair_ = {};
        }
      }
    }
    return *this;
  }

  /* Destructor. We can't use the `= default` due to our `union`. */
  ~MonomialVariableToDegreeMap() {
    if (is_multivariate()) {
      clear();
      // When size() == 0, the VariableDegreePair dtor is a no-op.
    } else {
      single_pair_.~VariableDegreePair();
    }
  }

  /* (Internal use only) Returns the total degree (the sum of each variable's
  individiual degree). */
  int total_degree() const {
    return is_multivariate() ? int{multivariate_.total_degree}
                             : single_pair_.second;
  }

  /* (Internal use only) Increments the degree of the given variable by 1.
  If it did not exist yet, then it is added instead (with degree 1).
  Returns the new degree. */
  int Increment(const Variable& var);

  /* (Internal use only) Decrements the degree of the given variable by 1.
  It is an error is the variable didn't exist (had a degree of zero).
  Returns the new degree. */
  int Decrement(const Variable& var);

  /* (Internal use only) Multiplies this by `other`. */
  void MultiplyInPlace(const MonomialVariableToDegreeMap& other);

  /* (Internal use only) Raises this to the `exponent` power.
  The exponent must be non-negative, but can be zero. */
  void PowInPlace(int exponent);

  // --------------------------------------------------------------------------
  // The remainder of the `public:` section is intended for users to rely on,
  // per the Monomial::get_power() documentation about std::map compatibility.
  // --------------------------------------------------------------------------

  // Element access like std::map (but const-only, so no operator[]).
  int at(const Variable& key) const;

  // Iterators like std::map (but const-only, and no reverse iterators).
  const_iterator begin() const { return cbegin(); }
  const_iterator end() const { return cend(); }
  const_iterator cbegin() const {
    return is_multivariate() ? multivariate_.sorted_pairs.data()
                             : &single_pair_;
  }
  const_iterator cend() const { return cbegin() + size(); }

  // Capacity like std::map (but const-only, and no max_size).
  bool empty() const { return size() == 0; }
  size_type size() const {
    return is_multivariate()         ? multivariate_.sorted_pairs.size()
           : single_pair_.second > 0 ? 1
                                     : 0;
  }

  // Modifiers like std::map. Only a few are provided because these are
  // effectively internal use only because users only ever have const access.
  void clear();

  // Lookup like std::map (but const-only).
  size_type count(const Variable& var) const {
    return find(var) != cend() ? 1 : 0;
  }
  const_iterator find(const Variable& var) const {
    if (is_multivariate()) {
      // TODO(jwnimmer-tri) This should not be inline.
      for (auto iter = cbegin(); iter != cend(); ++iter) {
        if (iter->first == var) {
          return iter;
        }
      }
      return cend();
    } else if (var == single_pair_.first) {
      return cbegin();
    } else {
      return cend();
    }
  }
  // TODO(jwnimmer-tri) equal_range, lower_bound, upper_bound

 private:
  bool is_multivariate() const { return tag_.is_multivariate; }

  struct Multivariate {
    // TODO(jwnimmer-tri) The copy, copy-assign, and dtor should not be inline.
    Tag tag{.is_multivariate = true};
    int total_degree{};
    std::vector<VariableDegreePair> sorted_pairs;
  };
  union {
    Tag tag_;
    VariableDegreePair single_pair_;
    Multivariate multivariate_;
  };
};

}  // namespace internal
}  // namespace symbolic
}  // namespace drake

#if !defined(DRAKE_DOXYGEN_CXX)
namespace std {
// Support for structured bindings.
template <>
struct tuple_size<
    drake::symbolic::internal::MonomialVariableToDegreeMap::VariableDegreePair>
    : std::integral_constant<size_t, 2> {};
template <size_t Index>
struct tuple_element<
    Index,
    drake::symbolic::internal::MonomialVariableToDegreeMap::VariableDegreePair>
    : std::conditional<Index == 0, drake::symbolic::Variable, int> {
  static_assert(Index < 2);
};
}  // namespace std
#endif
