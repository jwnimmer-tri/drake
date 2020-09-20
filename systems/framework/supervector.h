#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// Supervector is a concrete class template that implements VectorBase by
/// concatenating multiple VectorBases, which it does not own.
///
/// @tparam_default_scalar
template <typename T>
class Supervector final : public VectorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Supervector)

  /// Constructs a supervector consisting of all the vectors in subvectors,
  /// which must live at least as long as this supervector.
  explicit Supervector(const std::vector<VectorBase<T>*>& subvectors) {
    for (VectorBase<T>* vec : subvectors) {
      if (vec == nullptr) {
        throw std::logic_error("Cannot create Supervector of a nullptr.");
      }
      const int sub_n = vec->size();
      for (int i = 0; i < sub_n; ++i) {
        T& element_ref = (*vec)[i];
        data_.push_back(&element_ref);
      }
    }
  }

  ~Supervector() final = default;

  int size() const final {
    return data_.size();
  }

  void CopyToPreSizedVector(EigenPtr<VectorX<T>> vec) const final {
    DRAKE_THROW_UNLESS(vec != nullptr);
    const int n = vec->size();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    for (int i = 0; i < n; ++i) {
      const T& datum = *(data_[i]);
      (*vec)[i] = datum;
    }
  }

  void SetFrom(const VectorBase<T>& value) final {
    const int n = value.size();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    for (int i = 0; i < n; ++i) {
      T& datum = *(data_[i]);
      datum = value[i];
    }
  }

  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) final {
    const int n = value.size();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    for (int i = 0; i < n; ++i) {
      T& datum = *(data_[i]);
      datum = value[i];
    }
  }

  void ScaleAndAddToVector(const T& scale,
                           EigenPtr<VectorX<T>> vec) const final {
    DRAKE_THROW_UNLESS(vec != nullptr);
    const int n = vec->rows();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    for (int i = 0; i < n; ++i) {
      const T& datum = *(data_[i]);
      (*vec)[i] += scale * datum;
    }
  }

 private:
  using ScaledVectorInitList = typename VectorBase<T>::ScaledVectorInitList;

  const T& DoGetAtIndexUnchecked(int index) const final {
    DRAKE_ASSERT(index < size());
    return *(data_[index]);
  }

  T& DoGetAtIndexChecked(int index) final {
    DRAKE_ASSERT(index < size());
    return *(data_[index]);
  }

  const T& DoGetAtIndexChecked(int index) const final {
    if (index >= size()) { this->ThrowOutOfRange(index); }
    return *(data_[index]);
  }

  T& DoGetAtIndexChecked(int index) final {
    if (index >= size()) { this->ThrowOutOfRange(index); }
    return *(data_[index]);
  }

  void DoPlusEqScaled(const ScaledVectorInitList& rhs_scale) final {
    const int n = size();
    for (int i = 0; i < n; ++i) {
      T value(0);
      for (const auto& [scale, rhs] : rhs_scale) {
        value += rhs[i] * scale;
      }
      T& datum = *(data_[i]);
      datum += value;
    }
  }

  std::vector<T*> data_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Supervector)
