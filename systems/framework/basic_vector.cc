#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
BasicVector<T>::~BasicVector() = default;

template <typename T>
std::unique_ptr<BasicVector<T>> BasicVector<T>::Clone() const {
  auto clone = std::unique_ptr<BasicVector<T>>(DoClone());
  clone->set_value(this->get_value());
  return clone;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::BasicVector)
