#pragma once

// Depending on which translation unit we're being included into, the build
// settings in effect for Eigen might be different (e.g., SIMD-enabled vs not).
//
// TODO(jwnimmer-tri) -- FIXME
// Therefore, we must not include anything here that transitively includes
// Eigen. That mostly means using only C++ standard library, plus a few hand-
// picked tiny Drake headers that we know are safe.
//
// For the same reason, in this header we also require that all uses of Eigen
// must be flagged with DontAlign, because Eigen's memory alignment can change
// based on its build settings.

#include <type_traits>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/systems/primitives/perceptron_activation_type.h"

namespace drake {
namespace systems {
namespace internal {

/* Like MatrixX or VectorX, but opted-in to DontAlign. */
template <typename Scalar, int Cols = Eigen::Dynamic>
using AbiSafeMatrix =
    Eigen::Matrix<Scalar, Eigen::Dynamic, Cols, Eigen::DontAlign>;

/* Encapsulates calculation functions and scratch storage for one MLP layer. */
template <typename T>
class Layer final {
 public:
  explicit Layer(int layer_size);

  Layer(const Layer&);
  Layer& operator=(const Layer&);
  Layer(Layer&&);
  Layer& operator=(Layer&&);

  ~Layer();

  /* Calculates x₀ = f(x). This function should only be used on the 0th layer.
  When `use_sin_cos_for_input` is null, `f` is the identity.
  Otherwise, for any of its indices `i` that are `true`, x[i] is converted
  to a (sin, cos) pair in x₀.
  Typically only called on the input layer. */
  void CalcInputFeatures(const Eigen::Ref<const AbiSafeMatrix<T>>& X,
                         const std::vector<bool>* use_sin_cos_for_input);

  /* Calculates xₙ = σ(Wₙ₋₁xₙ₋₁+bₙ₋₁).
  The `antecedent` is the ₙ₋₁'th layer.
  The `W` and `b` are the factors beteween ₙ₋₁'th and ₙ'th layer.
  If `gradient` is true, also calculates dXn_dWx_plus_b.
  Typically only called on a hidden or final layer. */
  void CalcForward(const Layer& antecedent,
                   const Eigen::Ref<const AbiSafeMatrix<T>>& W,
                   const Eigen::Ref<const AbiSafeMatrix<T, 1>>& b,
                   PerceptronActivationType type, bool calc_gradient);

  /* XXX
  Typically only called on the final layer. */
  const AbiSafeMatrix<T>& get_output() const { return Xn_; }

  /* XXX
  Uses for backprop.
  Typically only called on the final layer. */
  void set_dloss_dXn(const Eigen::Ref<const AbiSafeMatrix<T>>& Y);

  /* XXX
  Typically only called on a hidden or final layer. */
  void CalcBackward(const Eigen::Ref<const AbiSafeMatrix<T>>& W,
                    Layer* antecedent);

  /* XXX
  Used for backprop.
  Typically only called on the input layer. */
  const AbiSafeMatrix<T>& get_dloss_dXn() const { return dloss_dXn_; }

 private:
  int layer_size_{};
  // TODO(jwnimmer-tri) We probably don't need abi-safe here.
  AbiSafeMatrix<T> Wx_;
  AbiSafeMatrix<T> Wx_plus_b_;
  AbiSafeMatrix<T> Xn_;
  AbiSafeMatrix<T> dXn_dWx_plus_b_;
  AbiSafeMatrix<T> dloss_dXn_;
  AbiSafeMatrix<T> dloss_dWx_plus_b_;
  AbiSafeMatrix<T> dloss_dW_;
  AbiSafeMatrix<T, 1> dloss_db_;
  AbiSafeMatrix<T> dloss_dinput_features_;
};

}  // namespace internal
}  // namespace systems
}  // namespace drake
