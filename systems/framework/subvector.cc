#include "drake/systems/framework/subvector.h"

#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"

namespace drake {
namespace systems {

template <typename T>
Subvector<T>::Subvector(
    VectorBase<T>* vector, int first_element, int num_elements)
    : vector_(vector),
      first_element_(first_element),
      num_elements_(num_elements) {
  if (vector == nullptr) {
    throw std::logic_error("Cannot create Subvector of a nullptr vector.");
  }
  if ((first_element < 0) || (num_elements < 0) ||
      (first_element + num_elements > vector->size())) {
    throw std::logic_error(fmt::format(
        "Subvector range [{}, {}) falls outside the valid range [{}, {}).",
        first_element, first_element + num_elements, 0, vector->size()));
  }
}

template <typename T>
Subvector<T>::Subvector(VectorBase<T>* vector)
    : Subvector(vector, 0, 0) {}

template <typename T>
Subvector<T>::~Subvector() = default;

template <typename T>
const T& Subvector<T>::DoGetAtIndex(int index) const {
  if (index >= size()) { this->ThrowOutOfRange(index); }
  return (*vector_)[first_element_ + index];
}

template <typename T>
T& Subvector<T>::DoGetAtIndex(int index) {
  if (index >= size()) { this->ThrowOutOfRange(index); }
  return (*vector_)[first_element_ + index];
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Subvector)
