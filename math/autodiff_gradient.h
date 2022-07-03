/** @file
Utilities that relate simultaneously to both autodiff matrices and
gradient matrices. */

#pragma once

#include <algorithm>
#include <optional>

#include <Eigen/Dense>
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/math/gradient.h"

namespace drake {
namespace math {

/** Extracts the `derivatives()` portion from a matrix of autodiff::Scalar
entries. (Each entry contains a value and derivatives.)

@param auto_diff_matrix An object whose Eigen type represents a matrix of
    autodiff::Scalar entries.
@param num_derivatives (Optional) The number of derivatives to return in case
    the input matrix has none, which we interpret as `num_derivatives` zeroes.
    If `num_derivatives` is supplied and the input matrix has derivatives, the
    sizes must match.
@retval gradient_matrix An Eigen::Matrix with number of rows equal to the
    total size (rows x cols) of the input matrix and number of columns equal
    to the number of derivatives. Each output row corresponds to one entry of
    the input matrix, in input row order.

@tparam Derived An Eigen type representing a matrix with autodiff::Scalar
    entries. The type will be inferred from the type of the `auto_diff_matrix`
    parameter at the call site.

@throws std::exception if the input matrix has elements with inconsistent,
    non-zero numbers of derivatives.
@throws std::exception if `num_derivatives` is specified but the input matrix
    has a different, non-zero number of derivatives.*/
template <typename Derived>
auto ExtractGradient(const Eigen::MatrixBase<Derived>& auto_diff_matrix,
                    std::optional<int> num_derivatives = {}) {
  // Entries in an AutoDiff matrix must all have the same number of derivatives,
  // or 0-length derivatives in which case they are interpreted as all-zero.
  int num_derivatives_from_matrix = 0;
  for (int i = 0; i < auto_diff_matrix.size(); ++i) {
    const int entry_num_derivs =
        static_cast<int>(auto_diff_matrix(i).derivatives().size());
    if (entry_num_derivs == 0) continue;  // Always OK.
    if (num_derivatives_from_matrix != 0 &&
        entry_num_derivs != num_derivatives_from_matrix) {
      throw std::logic_error(fmt::format(
          "ExtractGradient(): Input matrix has elements with inconsistent,"
          " non-zero numbers of derivatives ({} and {}).",
          num_derivatives_from_matrix, entry_num_derivs));
    }
    num_derivatives_from_matrix = entry_num_derivs;
  }

  if (!num_derivatives.has_value()) {
    num_derivatives = num_derivatives_from_matrix;
  } else if (num_derivatives_from_matrix != 0 &&
             num_derivatives_from_matrix != *num_derivatives) {
    throw std::logic_error(fmt::format(
        "ExtractGradient(): Input matrix has {} derivatives, but"
        " num_derivatives was specified as {}. Either the input matrix should"
        " have zero derivatives, or the number should match num_derivatives.",
        num_derivatives_from_matrix, *num_derivatives));
  }

  Eigen::Matrix<double, Derived::SizeAtCompileTime, Eigen::Dynamic>
      gradient(auto_diff_matrix.size(), *num_derivatives);
  if (gradient.size() == 0) {
    return gradient;
  }
  for (int row = 0; row < auto_diff_matrix.rows(); ++row) {
    for (int col = 0; col < auto_diff_matrix.cols(); ++col) {
      auto gradient_row =
          gradient.row(row + col * auto_diff_matrix.rows()).transpose();
      if (auto_diff_matrix(row, col).derivatives().size() == 0) {
        gradient_row.setZero();
      } else {
        gradient_row = auto_diff_matrix(row, col).derivatives();
      }
    }
  }
  return gradient;
}

/** Initializes an AutoDiff matrix given a matrix of values and a gradient
matrix.

@param[in] value The value matrix. Will be accessed with a single index.
@param[in] gradient The gradient matrix. The number of rows must match the
    total size (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.
@param[out] auto_diff_matrix The matrix of autodiff::Scalars. Will be resized as
    needed to have the same dimensions as the value matrix.
@exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
template <typename DerivedValue, typename DerivedGradient,
          typename DerivedAutoDiff>
void InitializeAutoDiff(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient,
    Eigen::MatrixBase<DerivedAutoDiff>* auto_diff_matrix) {
  DRAKE_THROW_UNLESS(auto_diff_matrix != nullptr);

  // The element types of the arguments must be as expected.
  static_assert(std::is_same_v<typename DerivedValue::Scalar, double>,
                "wrong value scalar type");
  static_assert(std::is_same_v<typename DerivedGradient::Scalar, double>,
                "wrong gradient scalar type");
  static_assert(std::is_same_v<typename DerivedAutoDiff::Scalar, AutoDiffXd>,
                "wrong auto diff matrix scalar type");

  // The compile-time dimensions of the arguments must match.
  static_assert(static_cast<int>(DerivedValue::SizeAtCompileTime) ==
                    static_cast<int>(DerivedGradient::RowsAtCompileTime),
                "gradient has wrong number of rows at compile time");
  static_assert(static_cast<int>(DerivedValue::RowsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::RowsAtCompileTime),
                "auto diff matrix has wrong number of rows at compile time");
  static_assert(static_cast<int>(DerivedValue::ColsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::ColsAtCompileTime),
                "auto diff matrix has wrong number of columns at compile time");

  // The runtime dimensions of the arguments must match.
  DRAKE_THROW_UNLESS(value.size() == gradient.rows() &&
      "gradient has wrong number of rows at runtime");

  auto& auto_diff = auto_diff_matrix->derived();
  auto_diff.resize(value.rows(), value.cols());
  for (Eigen::Index i = 0; i < auto_diff.size(); ++i) {
    auto_diff(i) = { value(i), gradient.row(i).transpose() };
  }
}

/** Returns an AutoDiff matrix given a matrix of values and a gradient
matrix.

@param[in] value The value matrix. Will be accessed with a single index.
@param[in] gradient The gradient matrix. The number of rows must match the
    total size (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.
@retval auto_diff_matrix The matrix of autodiff::Scalars. Will have the same
    dimensions as the value matrix.
@pydrake_mkdoc_identifier{value_and_gradient} */
template <typename DerivedValue, typename DerivedGradient>
auto InitializeAutoDiff(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient) {
  MatrixLikewise<AutoDiffXd, DerivedValue> auto_diff_matrix(
      value.rows(), value.cols());
  InitializeAutoDiff(value, gradient, &auto_diff_matrix);
  return auto_diff_matrix;
}

/** `B = DiscardZeroGradient(A, precision)` enables casting from a matrix of
autodiff::Scalars to double, but first checking that
the gradient matrix is empty or zero.  For a matrix of type, e.g.
`MatrixX<AutoDiffXd> A`, the comparable operation
  `B = A.cast<double>()`
should (and does) fail to compile.  Use `DiscardZeroGradient(A)` if you want
to force the cast (and the check).

XXX reference warning

See ExtractValue() for a note on similar Drake functions.

@param precision is passed to Eigen's isZero(precision) to evaluate whether
the gradients are zero.
@throws std::exception if the gradients were not empty nor zero.
@see DiscardGradient() */
template <typename Derived>
decltype(auto) DiscardZeroGradient(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix,
    double precision = Eigen::NumTraits<double>::dummy_precision()) {
  if constexpr (std::is_same_v<typename Derived::Scalar, double>) {
    unused(precision);
    return auto_diff_matrix;
  } else {
    const auto& gradients = ExtractGradient(auto_diff_matrix);
    if (gradients.size() == 0 || gradients.isZero(precision)) {
      return ExtractValue(auto_diff_matrix);
    }
    throw std::runtime_error(
        "Casting AutoDiff to value but gradients are not zero.");
  }
}

/**
 * Given a matrix of autodiff::Scalars, returns the size of the
 * derivatives.
 * @throw runtime_error if some entry has different (non-zero) number of
 * derivatives as the others.
 */
template <typename Derived>
typename std::enable_if<!std::is_same_v<typename Derived::Scalar, double>,
                        int>::type
GetDerivativeSize(const Eigen::MatrixBase<Derived>& A) {
  int num_derivs = 0;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      if (A(i, j).derivatives().size() != 0) {
        if (num_derivs != 0 && A(i, j).derivatives().size() != num_derivs) {
          throw std::runtime_error(fmt::format(
              "GetDerivativeSize(): A({}, {}).derivatives() has size "
              "{}, while another entry has size {}",
              i, j, A(i, j).derivatives().size(), num_derivs));
        }
        num_derivs = A(i, j).derivatives().size();
      }
    }
  }
  return num_derivs;
}

}  // namespace math
}  // namespace drake
