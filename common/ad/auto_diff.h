#pragma once

#include "drake/common/ad/internal/partials.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt.h"

namespace drake {
namespace ad {

/** A scalar type that performs automatic differentiation, similar to
`Eigen::AutoDiffScalar<Eigen::VectorXd>`. Unlike `Eigen::AutoDiffScalar`,
Drake's AutoDiff is not templated; it only supports dynamically-sized
derivatives using floating-point doubles.

At the moment, the features of this class are similar to Eigen::AutoDiffScalar,
but in the future Drake will further customize this class to optimize its
runtime performance and better integrate it with our optimization tools. */
class AutoDiff {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AutoDiff);

  /** Compatibility alias to mimic Eigen::AutoDiffScalar. */
  using DerType = Eigen::VectorXd;

  /** Compatibility alias to mimic Eigen::AutoDiffScalar. */
  using Scalar = double;

  /** Constructs zero. */
  AutoDiff() = default;

  /** Constructs a value with empty derivatives. */
  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  AutoDiff(double value) : value_{value} {}

  /** Constructs a value with a single partial derivative of 1.0 at the given
  `offset` in a vector of `size` otherwise-zero derivatives. */
  AutoDiff(double value, Eigen::Index size, Eigen::Index offset)
      : value_{value}, partials_{size, offset} {}

  /** Constructs a value with the given derivatives. */
  AutoDiff(double value, const Eigen::Ref<const Eigen::VectorXd>& derivatives)
      : value_{value}, partials_{derivatives} {}

  /** Assigns a value and clears the derivatives. */
  AutoDiff& operator=(double value) {
    value_ = value;
    partials_.SetZero();
    return *this;
  }

  ~AutoDiff() = default;

  /** Returns the value part of this %AutoDiff (readonly). */
  double value() const { return value_; }

  /** (Advanced) Returns the value part of this %AutoDiff (mutable).
  Operations on this value will NOT alter the derivatives. */
  double& value() { return value_; }

  /** Returns a view of the derivatives part of this %AutoDiff (readonly).

  Do not presume any specific C++ type for the the return value. It will act
  like an Eigen column-vector expression (e.g., Eigen::Block<const VectorXd>),
  but we reserve the right to change the return type for efficiency down the
  road. */
  const Eigen::VectorXd& derivatives() const {
    return partials_.make_const_xpr();
  }

  /** (Advanced) Returns a mutable view of the derivatives part of this
  %AutoDiff.

  Instead of mutating the derivatives after construction, it's generally
  preferable to set them directly in the constructor if possible.

  Do not presume any specific C++ type for the the return value. It will act
  like a mutable Eigen column-vector expression (e.g., Eigen::Block<VectorXd>)
  that also allows for assignment and resizing, but we reserve the right to
  change the return type for efficiency down the road. */
  Eigen::VectorXd& derivatives() { return partials_.get_raw_storage_mutable(); }

  /// @name Internal use only
  //@{

  /** (Internal use only)
  Users should call derivatives() instead. */
  const internal::Partials& partials() const { return partials_; }

  /** (Internal use only)
  Users should call derivatives() instead. */
  internal::Partials& partials() { return partials_; }

  //@}

 private:
  double value_{0.0};
  internal::Partials partials_;
};

}  // namespace ad

/** Returns the autodiff scalar's value() as a double. Never throws. Overloads
ExtractDoubleOrThrow() from `drake/common/extract_double.h`.
@see math::ExtractValue(), math::DiscardGradient() */
inline double ExtractDoubleOrThrow(const ad::AutoDiff& scalar) {
  return scalar.value();
}

/** Returns `matrix` as an Eigen::Matrix<double, ...> with the same size
allocation as `matrix`. Calls ExtractDoubleOrThrow on each element of the
matrix, and therefore throws if any one of the extractions fail.
@see math::ExtractValue(), math::DiscardGradient() */
template <int RowsAtCompileTime, int ColsAtCompileTime, int Options,
          int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
auto ExtractDoubleOrThrow(
    const Eigen::MatrixBase<
        Eigen::Matrix<ad::AutoDiff, RowsAtCompileTime, ColsAtCompileTime,
                      Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>>&
        matrix) {
  return matrix
      .unaryExpr([](const ad::AutoDiff& value) {
        return ExtractDoubleOrThrow(value);
      })
      .eval();
}

/** Provides if-then-else expression for AutoDiff. */
inline ad::AutoDiff if_then_else(bool f_cond, const ad::AutoDiff& x,
                                 const ad::AutoDiff& y) {
  return f_cond ? x : y;
}

/** Provides cond expression for AutoDiff. */
template <typename... Rest>
inline ad::AutoDiff cond(bool f_cond, const ad::AutoDiff& e_then,
                         Rest... rest) {
  return if_then_else(f_cond, e_then, cond(rest...));
}

}  // namespace drake

// These further refine our AutoDiff type and must appear in exactly this order.
// clang-format off
#include "drake/common/ad/internal/standard_operations.h"
#include "drake/common/ad/internal/eigen_specializations.h"
// clang-format on

/* Formats the `value()` part of x to the stream.
To format the derivatives use `drake::fmt_eigen(x.derivatives())`. */
DRAKE_FORMATTER_AS(, drake::ad, AutoDiff, x, x.value())
