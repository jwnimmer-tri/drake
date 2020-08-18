#pragma once

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
  explicit Supervector(const std::vector<VectorBase<T>*>& subvectors);

  ~Supervector() final;

  int size() const final {
    return lookup_table_.back();
  }

 private:
  const T& DoGetAtIndex(int index) const final;
  T& DoGetAtIndex(int index) final;

  std::pair<VectorBase<T>*, int> GetSubvectorAndOffset(int index) const;

  // An ordered list of all the constituent vectors in this supervector.
  std::vector<VectorBase<T>*> vectors_;

  // The integer in the lookup_table_ at index N is the sum of the number of
  // elements in the constituent vectors 0 through N inclusive.
  // For example, if the sizes of the constituent vectors are [1, 3, 5],
  // the lookup table is [1, 4, 9].
  std::vector<int> lookup_table_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Supervector)
