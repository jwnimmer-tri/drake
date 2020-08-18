#include "drake/systems/framework/vector_base.h"

#include <stdexcept>

#include "drake/common/drake_throw.h"

namespace drake {
namespace systems {

template <typename T>
VectorBase<T>::~VectorBase() = default;

template <typename T>
void VectorBase<T>::SetFrom(const VectorBase<T>& value) {
  const int n = value.size();
  if (n != size()) {
    ThrowMismatchedSize(n);
  }
  for (int i = 0; i < n; ++i) {
    (*this)[i] = value[i];
  }
}

template <typename T>
void VectorBase<T>::SetFromVector(const Eigen::Ref<const VectorX<T>>& value) {
  const int n = value.rows();
  if (n != size()) {
    ThrowMismatchedSize(n);
  }
  for (int i = 0; i < n; ++i) {
    (*this)[i] = value[i];
  }
}

template <typename T>
void VectorBase<T>::SetZero() {
  const int n = size();
  for (int i = 0; i < n; ++i) {
    (*this)[i] = T(0.0);
  }
}

template <typename T>
VectorX<T> VectorBase<T>::CopyToVector() const {
  const int n = size();
  VectorX<T> vec(n);
  for (int i = 0; i < n; ++i) {
    vec[i] = (*this)[i];
  }
  return vec;
}

template <typename T>
void VectorBase<T>::CopyToPreSizedVector(EigenPtr<VectorX<T>> vec) const {
  DRAKE_THROW_UNLESS(vec != nullptr);
  const int n = vec->rows();
  if (n != size()) {
    ThrowMismatchedSize(n);
  }
  for (int i = 0; i < n; ++i) {
    (*vec)[i] = (*this)[i];
  }
}

template <typename T>
void VectorBase<T>::ScaleAndAddToVector(const T& scale,
                                        EigenPtr<VectorX<T>> vec) const {
  DRAKE_THROW_UNLESS(vec != nullptr);
  const int n = vec->rows();
  if (n != size()) {
    ThrowMismatchedSize(n);
  }
  for (int i = 0; i < n; ++i) {
    (*vec)[i] += scale * (*this)[i];
  }
}

template <typename T>
void VectorBase<T>::GetElementBounds(Eigen::VectorXd* lower,
                                     Eigen::VectorXd* upper) const {
  lower->resize(0);
  upper->resize(0);
}

template <typename T>
void VectorBase<T>::DoPlusEqScaled(const std::initializer_list<
                                   std::pair<T, const VectorBase<T>&>>& rhs_scale) {
  const int n = size();
  for (int i = 0; i < n; ++i) {
    T value(0);
    for (const auto& operand : rhs_scale) {
      value += operand.second[i] * operand.first;
    }
    (*this)[i] += value;
  }
}

template <typename T>
void VectorBase<T>::ThrowOutOfRange(int index) const {
  // XXX
  throw std::out_of_range("Index " + std::to_string(index) +
                          " out of bounds for supervector of size " +
                          std::to_string(size()));
}

template <typename T>
void VectorBase<T>::ThrowMismatchedSize(int other_size) const {
  // XXX
  throw std::out_of_range(
      "Cannot set a BasicVector of size " + std::to_string(size()) +
      " with a value of size " + std::to_string(other_size));
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorBase)
