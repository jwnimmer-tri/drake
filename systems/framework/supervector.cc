#include "drake/systems/framework/supervector.h"

#include <algorithm>

namespace drake {
namespace systems {

template <typename T>
Supervector<T>::Supervector(const std::vector<VectorBase<T>*>& subvectors)
    : vectors_(subvectors) {
  int sum = 0;
  for (const VectorBase<T>* vec : vectors_) {
    if (vec == nullptr) {
      throw std::logic_error("Cannot create Supervector of a nullptr vector.");
    }
    sum += vec->size();
    lookup_table_.push_back(sum);
  }
  if (lookup_table_.empty()) {
    // Enable this->size() to unconditionally call back() on the lookup table.
    lookup_table_.push_back(0);
  }
}

template <typename T>
Supervector<T>::~Supervector() = default;

// Given an index into the supervector, returns the subvector that contains
// that index, and its offset within the subvector. This operation is O(log(N))
// in the number of subvectors. Throws std::out_of_range for invalid indices.
//
// Example: if the lookup table is [1, 4, 9], and @p index is 5, this function
// returns a pointer to the third of three subvectors, with offset 1, because
// the element at index 5 in the supervector is at index 1 in that subvector.
//
// 0 | 1 2 3 | 4 5 6 7 8
//               ^ index 5
template <typename T>
std::pair<VectorBase<T>*, int>
Supervector<T>::GetSubvectorAndOffset(int index) const {
  if (index >= size()) {
    this->ThrowOutOfRange(index);
  }

  // Binary-search the lookup_table_ for the first element that is larger than
  // the specified index.
  const auto it =
      std::upper_bound(lookup_table_.begin(), lookup_table_.end(), index);

  // Use the lookup result to identify the subvector that contains the index.
  const int subvector_id =
      static_cast<int>(std::distance(lookup_table_.begin(), it));
  VectorBase<T>* subvector = vectors_[subvector_id];

  // The item at index 0 in vectors_[subvector_id] corresponds to index
  // lookup_table_[subvector_id - 1] in the supervector.
  const int start_of_subvector = (subvector_id == 0) ? 0 : *(it - 1);
  return std::make_pair(subvector, index - start_of_subvector);
}

template <typename T>
const T& Supervector<T>::DoGetAtIndex(int index) const {
  const auto [subvector_ptr, offset] = GetSubvectorAndOffset(index);
  return (*subvector_ptr)[offset];
}

template <typename T>
T& Supervector<T>::DoGetAtIndex(int index) {
  const auto [subvector_ptr, offset] = GetSubvectorAndOffset(index);
  return (*subvector_ptr)[offset];
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Supervector)
