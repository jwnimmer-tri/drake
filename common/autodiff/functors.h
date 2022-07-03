#pragma once

#include <type_traits>
#include <utility>

#include <Eigen/Core>

#include "drake/common/autodiff/scalar.h"

namespace Eigen {
namespace internal {
#define DRAKE_ADS drake::autodiff::Scalar

// === See AssignmentFunctors.h ===

template <>
struct assign_op<DRAKE_ADS, DRAKE_ADS> {
  void assignCoeff(DRAKE_ADS& a, const DRAKE_ADS& b) const { a = b; }
  void assignCoeff(DRAKE_ADS& a, DRAKE_ADS&& b) const { a = std::move(b); }
};

// === See UnaryFunctors.h ===

template <>
struct scalar_opposite_op<DRAKE_ADS> {
  DRAKE_ADS operator()(DRAKE_ADS a) const { return -std::move(a); }
};

template <>
struct scalar_conjugate_op<DRAKE_ADS> {
  DRAKE_ADS operator()(const DRAKE_ADS& a) const { return a; }
  DRAKE_ADS operator()(DRAKE_ADS&& a) const { return std::move(a); }
};

// === See BinaryFunctors.h ===

template <>
struct scalar_sum_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a += b;
    return a;
  }
};

template <>
struct scalar_product_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a *= b;
    return a;
  }
};

template <>
struct scalar_difference_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a -= b;
    return a;
  }
};

template <>
struct scalar_quotient_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a /= b;
    return a;
  }
};

template <>
struct scalar_min_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator() (DRAKE_ADS a, const DRAKE_ADS& b) const {
    return min(std::move(a), b);
  }
};

template <>
struct scalar_max_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator() (DRAKE_ADS a, const DRAKE_ADS& b) const {
    return max(std::move(a), b);
  }
};

// === See Redux.h ===

template <typename Derived>
struct redux_impl<
    scalar_sum_op<DRAKE_ADS, DRAKE_ADS>,
    Derived,
    /* Traversal = */ DefaultTraversal,
    /* Unrolling = */ NoUnrolling> {
  using Scalar = DRAKE_ADS;
  template <typename Func>
  static Scalar run(const Derived& mat, const Func& func)
  {
    DRAKE_ASSERT(mat.size() > 0);
    Scalar result = mat.coeffByOuterInner(0, 0);
    for (Index i = 1; i < mat.innerSize(); ++i)
      result = func(std::move(result), mat.coeffByOuterInner(0, i));
    for (Index i = 1; i < mat.outerSize(); ++i)
      for (Index j = 0; j < mat.innerSize(); ++j)
        result = func(std::move(result), mat.coeffByOuterInner(i, j));
    return result;
  }

};

#undef DRAKE_ADS
}  // namespace internal
}  // namespace Eigen

