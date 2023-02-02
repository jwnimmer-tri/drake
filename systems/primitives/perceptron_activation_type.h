#pragma once

namespace drake {
namespace systems {

/** Specifies one of the common activation functions in a neural network. */
enum PerceptronActivationType {
  kIdentity,
  kReLU,
  kTanh,
};

}  // namespace systems
}  // namespace drake
