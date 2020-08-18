#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// Subvector is a concrete class template that implements VectorBase by
/// providing a sliced view of another VectorBase.
///
/// @tparam_default_scalar
template <typename T>
class Subvector final : public VectorBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Subvector)

  /// Constructs a subvector of `vector` that consists of `num_elements`
  /// starting at `first_element`.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  Subvector(VectorBase<T>* vector, int first_element, int num_elements);

  /// Constructs an empty subvector.
  /// @param vector The vector to slice.  Must not be nullptr. Must remain
  ///               valid for the lifetime of this object.
  explicit Subvector(VectorBase<T>* vector);

  ~Subvector() final;

  int size() const final { return num_elements_; }

 private:
  const T& DoGetAtIndex(int index) const final;
  T& DoGetAtIndex(int index) final;

  VectorBase<T>* const vector_;
  const int first_element_;
  const int num_elements_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Subvector)
