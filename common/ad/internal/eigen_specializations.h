#pragma once

#include <limits>
#include <utility>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"

/* This file contains Eigen-related specializations for Drake's AutoDiff
scalar type (plus the intimately related std::numeric_limits specialization).

The specializations both add basic capability (e.g., NumTraits) as well as
improve performance (e.g., opt-in to rvalue moves instead of copies).

NOTE: This file should never be included directly, rather only from
auto_diff.h in a very specific order. */

#ifndef DRAKE_DOXYGEN_CXX

namespace std {
template <>
class numeric_limits<drake::ad::AutoDiff> : public numeric_limits<double> {};
}  // namespace std

namespace Eigen {

// === See Eigen/src/Core/NumTraits.h ===

// See https://eigen.tuxfamily.org/dox/structEigen_1_1NumTraits.html.
// We'll Inherit the constants from `double`, but be sure to fatten a few types
// up to full AutoDiff where necessary.
template <>
struct NumTraits<drake::ad::AutoDiff> : public NumTraits<double> {
  // This refers to the "real part" of a complex number (e.g., std::complex).
  // Because we're not a complex number, it's just the same type as ourselves.
  using Real = drake::ad::AutoDiff;

  // This promotes integer types during operations like quotients, square roots,
  // etc. We're already floating-point, so it's just the same type as ourselves.
  using NonInteger = drake::ad::AutoDiff;

  // Eigen says "If you don't know what this means, just use [your type] here."
  using Nested = drake::ad::AutoDiff;

  // Our constructor is required during matrix storage initialization.
  enum { RequireInitialization = 1 };
};

// Computing "ADS [op] double" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<drake::ad::AutoDiff, double, BinOp> {
  using ReturnType = drake::ad::AutoDiff;
};

// Computing "double [op] ADS" yields an ADS.
template <typename BinOp>
struct ScalarBinaryOpTraits<double, drake::ad::AutoDiff, BinOp> {
  using ReturnType = drake::ad::AutoDiff;
};

namespace internal {

// An abbreviation makes this file much more readable (w.r.t. 80-col wrapping).
// We use "ADS" to stand for "AutoDiff scalar".
#define DRAKE_ADS drake::ad::AutoDiff

// === See Eigen/src/Core/functors/AssignmentFunctors.h ===

#if 0
// This specialization allows `b` to be moved-from rather than copied.
template <>
struct assign_op<DRAKE_ADS, DRAKE_ADS> {
  void assignCoeff(DRAKE_ADS& a, const DRAKE_ADS& b) const { a = b; }
  void assignCoeff(DRAKE_ADS& a, DRAKE_ADS&& b) const { a = std::move(b); }
};
#endif

// === See Eigen/src/Core/functors/UnaryFunctors.h ===

#if 0
// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_opposite_op<DRAKE_ADS> {
  DRAKE_ADS operator()(DRAKE_ADS a) const { return -std::move(a); }
};
#endif

#if 0
// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_conjugate_op<DRAKE_ADS> {
  DRAKE_ADS operator()(const DRAKE_ADS& a) const { return a; }
  DRAKE_ADS operator()(DRAKE_ADS&& a) const { return std::move(a); }
};
#endif

// === See Eigen/src/Core/functors/BinaryFunctors.h ===

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_sum_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a += b;
    return a;
  }
};

// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_product_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a *= b;
    return a;
  }
};

#if 0
// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_difference_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a -= b;
    return a;
  }
};
#endif

#if 0
// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_quotient_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    a /= b;
    return a;
  }
};
#endif

#if 0
// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_min_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    return drake::ad::min(std::move(a), b);
  }
};
#endif

#if 0
// This specialization allows `a` to be moved-from rather than copied.
template <>
struct scalar_max_op<DRAKE_ADS, DRAKE_ADS>
    : binary_op_base<DRAKE_ADS, DRAKE_ADS> {
  using result_type = DRAKE_ADS;
  DRAKE_ADS operator()(DRAKE_ADS a, const DRAKE_ADS& b) const {
    return drake::ad::max(std::move(a), b);
  }
};
#endif

// === See Eigen/src/Core/Redux.h ===

// This specialization reduces copying of `result` by adding `std::move`.
template <typename Evaluator>
struct redux_impl<scalar_sum_op<DRAKE_ADS, DRAKE_ADS>, Evaluator,
                  /* Traversal = */ DefaultTraversal,
                  /* Unrolling = */ NoUnrolling> {
  using Scalar = DRAKE_ADS;
  template <typename XprType>
  static DRAKE_ADS run(const Evaluator& eval,
                       const scalar_sum_op<DRAKE_ADS, DRAKE_ADS>& func,
                       const XprType& xpr) {
    DRAKE_ASSERT(xpr.size() > 0);
    DRAKE_ADS result = eval.coeffByOuterInner(0, 0);
    for (Index i = 1; i < xpr.innerSize(); ++i)
      result = func(std::move(result), eval.coeffByOuterInner(0, i));
    for (Index i = 1; i < xpr.outerSize(); ++i)
      for (Index j = 0; j < xpr.innerSize(); ++j)
        result = func(std::move(result), eval.coeffByOuterInner(i, j));
    return result;
  }
};

}  // namespace internal
}  // namespace Eigen

#undef DRAKE_ADS

#endif  // DRAKE_DOXYGEN_CXX
