#if 0
// clang-format off
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wold-style-cast"
// #include "drake_vendor/eigen_amalgamation.h"
// #pragma GCC diagnostic pop
#define Eigen drake_vendor::Eigen
#include <Eigen/Core>
using namespace drake_vendor;  // NOLINT
// clang-format on
#endif

#include "drake/systems/primitives/multilayer_perceptron_impl.h"

#include <limits>

#include "drake/common/drake_assert.h"

// XXX
#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {
namespace internal {

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}

template <typename T>
Layer<T>::Layer(int layer_size)
  : layer_size_(layer_size) {
  // TODO(jwnimmer-tri) I'm no convinced that preallocating intermediate storage
  // to placate LimitMalloc{.max = 0} is a good tradeoff vs blowing through our
  // L3 cache. We should benchmark this to see whether its actually useful.
  Wx_.setConstant(num_neurons, 1, kNaN);
  Wx_plus_b_.setConstant(num_neurons, 1, kNaN);
  Xn_.setConstant(num_neurons, 1, kNaN);
}

template <typename T>
Layer<T>::~Layer() = default;

template <typename T>
Layer<T>::Layer(const Layer<T>&) = default;

template <typename T>
Layer<T>& Layer<T>::operator=(const Layer<T>&) {
  // We are only scratch space, so it's fine for assignment to be a no-op.
  return *this;
}

template <typename T>
Layer<T>::Layer(Layer<T>&&) {
  // We are only scratch space, so it's fine for the move constructor to be the
  // same as the default constructor.
}

template <typename T>
Layer<T>& Layer<T>::operator=(Layer<T>&&) {
  // We are only scratch space, so it's fine for assignment to be a no-op.
  return *this;
}

template <typename T>
void Layer<T>::CalcInputFeatures(
    const Eigen::Ref<const AbiSafeMatrix<T>>& X,
    const std::vector<bool>* const use_sin_cos_for_input) {
  // If there are no input features; just use the identity.
  if (use_sin_cos_for_input == nullptr) {
    Xn_.noalias() = X;
    return;
  }

  // Calculate the input features.
  Xn_.resize(layer_size_, X.cols());
  int feature_row = 0, input_row = 0;
  for (bool use_sin_cos : *use_sin_cos_for_input) {
    if (use_sin_cos) {
      Xn_.row(feature_row++) = X.row(input_row).array().sin();
      Xn_.row(feature_row++) = X.row(input_row).array().cos();
    } else {
      Xn_.row(feature_row++) = X.row(input_row);
    }
    ++input_row;
  }
}

template <typename T>
void Layer<T>::CalcForward(const Layer& antecedent,
                           const Eigen::Ref<const AbiSafeMatrix<T>>& W,
                           const Eigen::Ref<const AbiSafeMatrix<T, 1>>& b,
                           const PerceptronActivationType type,
                           const bool calc_gradient) {
  Wx_.noalias() = W * antecedent.Xn_;
  Wx_plus_b_.noalias() = Wx_.colwise() + b;
  switch (type) {
    case kTanh: {
      Xn_.noalias() = Wx_plus_b_.array().tanh().matrix();
      if (calc_gradient) {
        dXn_dWx_plus_b_.noalias() =
            (1.0 - Wx_plus_b_.array().tanh().square()).matrix();
      }
      return;
    }
    case kReLU: {
      Xn_.noalias() = Wx_plus_b_.array().max(0.0).matrix();
      if (calc_gradient) {
        dXn_dWx_plus_b_.noalias() =
            (Wx_plus_b_.array() <= 0).select(0 * Wx_plus_b_, 1);
      }
      return;
    }
    case kIdentity: {
      Xn_.noalias() = Wx_plus_b_;
      if (calc_gradient) {
        dXn_dWx_plus_b_.setConstant(Wx_plus_b_.rows(), Wx_plus_b_.cols(), 1.0);
      }
      return;
    }
  }
  DRAKE_UNREACHABLE();
}

template <typename T>
void Layer<T>::set_dloss_dXn(const Eigen::Ref<const AbiSafeMatrix<T>>& Y) {
  dloss_dXn_ = Y;
}

template <typename T>
void Layer<T>::CalcBackward(const Eigen::Ref<const AbiSafeMatrix<T>>& W,
                            Layer* antecedent) {
  DRAKE_DEMAND(antecedent != nullptr);

  dloss_dWx_plus_b_ = (dloss_dXn_.array() * dXn_dWx_plus_b_.array()).matrix();
  antecedent->dloss_dXn_.noalias() = W.transpose() * dloss_dWx_plus_b_;

#if 0
  dloss_dWx_plus_b[i] =
    (dloss_dXn[i].array() * dXn_dWx_plus_b[i].array()).matrix();
  if (has_input_features_) {
    dloss_dinput_features.noalias() = W.transpose() * dloss_dWx_plus_b[0];
    int feature_row = 0, input_row = 0;
    for (bool use_sin_cos : use_sin_cos_for_input_) {
      if (use_sin_cos) {
        dYdX->row(input_row) =
            dloss_dinput_features.row(feature_row).array() *
                X.row(input_row).array().cos() -
            dloss_dinput_features.row(feature_row + 1).array() *
                X.row(input_row).array().sin();
        feature_row += 2;
        ++input_row;
      } else {
        dYdX->row(input_row++) = dloss_dinput_features.row(feature_row++);
      }
    }
  } else {
    dYdX->noalias() = W.transpose() * dloss_dWx_plus_b[i];
  }
#endif
}

template class Layer<double>;
template class Layer<AutoDiffXd>;
template class Layer<symbolic::Expression>;

}  // namespace internal
}  // namespace systems
}  // namespace drake
