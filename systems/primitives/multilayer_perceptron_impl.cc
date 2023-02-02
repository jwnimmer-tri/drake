// clang-format off
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#include "drake_vendor/eigen_amalgamation.h"
#pragma GCC diagnostic pop
using namespace drake_vendor;  // NOLINT
// clang-format on

#include "drake/systems/primitives/multilayer_perceptron_impl.h"

namespace drake {
namespace systems {
namespace internal {

__attribute__((flatten))
void double_operator_WX_plus_b::Calc() {
  Wx->noalias() = W * X;
  Wx_plus_b->noalias() = Wx->colwise() + b;
}

__attribute__((flatten))
void double_operator_Activation_tanh_Dynamic::Calc() {
  *Y = X.array().tanh().matrix();
  if (dYdX) {
    dYdX->noalias() = (1.0 - X.array().tanh().square()).matrix();
  }
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
