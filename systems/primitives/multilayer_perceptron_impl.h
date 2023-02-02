#pragma once

// In this file, we use Eigen types but we don't #include any Eigen headers.
// It's the responsibility of whoever includes us to include Eigen first.
// Depending on which translation unit we're being included into, the build
// settings in effect for Eigen might be different (e.g., SIMD-enabled vs not).
//
// Therefore, we must not include anything here that transitively includes
// Eigen. That mostly means using only C++ standard library, plus a few hand-
// picked tiny Drake headers that we know are safe.
//
// For the same reason, in this header we also require that all uses of Eigen
// must be flagged with DontAlign, because Eigen's memory alignment can change
// based on its build settings.

#include <type_traits>

#include "drake/common/drake_export.h"
#include "drake/systems/primitives/multilayer_perceptron_activation_type.h"

namespace drake {
namespace systems {
namespace internal {

/* Like MatrixX or VectorX, but opted-in to DontAlign. */
template <typename Scalar, int Cols = Eigen::Dynamic>
using AbiSafeMatrix =
    Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, Eigen::DontAlign>;

/* An optimized implementation of Calc_WX_plus_b<double>. We need to use a
helper struct for this (instead of a specialization) so that the mangled
function name does not mention Eigen at all. */
struct DRAKE_NO_EXPORT double_operator_WX_plus_b {
  const Eigen::Map<const AbiSafeMatrix<double>>& W;
  const AbiSafeMatrix<double>& X;
  const Eigen::Map<const AbiSafeMatrix<double, 1>>& b;
  AbiSafeMatrix<double>* Wx;
  AbiSafeMatrix<double>* Wx_plus_b;

  void Calc();
};

/* Calculates Wₙxₙ+bₙ (notably, without any activation).
@tparam_default_scalar */
template <typename T>
void Calc_WX_plus_b(
    const Eigen::Map<const AbiSafeMatrix<T>>& W,
    const AbiSafeMatrix<T>& X,
    const Eigen::Map<const AbiSafeMatrix<T, 1>>& b,
    AbiSafeMatrix<T>* Wx,
    AbiSafeMatrix<T>* Wx_plus_b) {
  if constexpr (std::is_same_v<T, double>) {
    const Eigen::Index Wx_rows = W.rows();
    const Eigen::Index Wx_cols = X.cols();
    Wx->resize(Wx_rows, Wx_cols);
    Wx_plus_b->resize(Wx_rows, Wx_cols);
    double_operator_WX_plus_b{W, X, b, Wx, Wx_plus_b}.Calc();
  } else {
    Wx->noalias() = W * X;
    Wx_plus_b->noalias() = Wx->colwise() + b;
  }
}

struct DRAKE_NO_EXPORT double_operator_Activation_tanh_Dynamic {
  const AbiSafeMatrix<double>& X;
  AbiSafeMatrix<double>* Y;
  AbiSafeMatrix<double>* dYdX;

  void Calc();
};

template <typename T, int cols>
void CalcActivation(
    PerceptronActivationType type,
    const Eigen::Matrix<T, Eigen::Dynamic, cols, Eigen::DontAlign>& X,
    Eigen::Matrix<T, Eigen::Dynamic, cols, Eigen::DontAlign>* Y,
    Eigen::Matrix<T, Eigen::Dynamic, cols, Eigen::DontAlign>* dYdX = nullptr) {
  Y->resize(X.rows(), X.cols());
  if (dYdX) {
    dYdX->resize(X.rows(), X.cols());
  }
  switch (type) {
    case kTanh: {
      if constexpr (std::is_same_v<T, double> && cols == Eigen::Dynamic) {
        double_operator_Activation_tanh_Dynamic{X, Y, dYdX}.Calc();
      } else {
        *Y = X.array().tanh().matrix();
        if (dYdX) {
          dYdX->noalias() = (1.0 - X.array().tanh().square()).matrix();
        }
      }
      return;
    }
    case kReLU: {
      *Y = X.array().max(0.0).matrix();
      if (dYdX) {
        dYdX->noalias() = (X.array() <= 0).select(0 * X, 1);
      }
      return;
    }
    case kIdentity: {
      *Y = X;
      if (dYdX) {
        dYdX->setConstant(1.0);
      }
      return;
    }
  }
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
