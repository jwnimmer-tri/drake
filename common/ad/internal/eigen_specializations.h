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

namespace drake {
namespace ad {
namespace internal {

/* Optimized implementations of BLAS GEMM for autodiff types to take advantage
of scalar type specializations. With our current mechanism for hooking this into
Eigen, we only need to support the simplified form C ⇐ A@B rather than the more
general C ⇐ αA@B+βC of typical GEMM; if we figure out how to hook into Eigen's
expression templates, we could expand to the more general form. We group these
functions using a struct so that the friendship declaration with XXXXXX can be
straightforward.
@tparam reverse When true, calculates B@A instead of A@B. */
template <bool reverse>
struct Gemm {
  Gemm() = delete;
  // Allow for passing numpy.ndarray without copies.
  template <typename T>
  using MatrixRef = Eigen::Ref<const MatrixX<T>, 0, StrideX>;
  // Matrix product for AutoDiff, AutoDiff.
  // When reverse == false, sets result to A * B.
  // When reverse == true, sets result to B * A.
  static void CalcAA(const MatrixRef<AutoDiff>& A, const MatrixRef<AutoDiff>& B,
                     EigenPtr<MatrixX<AutoDiff>> result);
};

}  // namespace internal

// Matrix<AutoDiff> * Matrix<AutoDiff> => Matrix<AutoDiff>
template <typename MatrixL, typename MatrixR>
typename std::enable_if_t<
    std::is_base_of_v<Eigen::MatrixBase<MatrixL>, MatrixL> &&
        std::is_base_of_v<Eigen::MatrixBase<MatrixR>, MatrixR> &&
        std::is_same_v<typename MatrixL::Scalar, AutoDiff> &&
        std::is_same_v<typename MatrixR::Scalar, AutoDiff>,
    Eigen::Matrix<AutoDiff, MatrixL::RowsAtCompileTime,
                  MatrixR::ColsAtCompileTime>>
operator*(const MatrixL& lhs, const MatrixR& rhs) {
  Eigen::Matrix<AutoDiff, MatrixL::RowsAtCompileTime,
                MatrixR::ColsAtCompileTime>
      result;
  DRAKE_THROW_UNLESS(lhs.cols() == rhs.rows());
  result.resize(lhs.rows(), rhs.cols());
  constexpr bool reverse = false;
  internal::Gemm<reverse>::CalcAA(lhs, rhs, &result);
  return result;
}

}  // namespace ad
}  // namespace drake

#endif  // DRAKE_DOXYGEN_CXX
