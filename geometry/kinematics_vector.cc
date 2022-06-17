#include "drake/geometry/kinematics_vector.h"

#include <algorithm>
#include <stdexcept>

#include "absl/container/flat_hash_map.h"

#include "drake/common/autodiff.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace geometry {

template <class Id, class KinematicsValue>
struct __attribute__((visibility("hidden")))
KinematicsVector<Id, KinematicsValue>::Impl {
  absl::flat_hash_map<Id, KinematicsValue> map;
};

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector() = default;

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    std::initializer_list<std::pair<const Id, KinematicsValue>> init)
    : KinematicsVector() {
  for (const auto& item : init) {
    set_value(item.first, item.second);
  }
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    std::initializer_list<std::pair<const Id, KinematicsValue>> init) {
  clear();
  for (const auto& item : init) {
    set_value(item.first, item.second);
  }
  return *this;
}

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    const KinematicsVector<Id, KinematicsValue>&) = default;

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::KinematicsVector(
    KinematicsVector<Id, KinematicsValue>&&) = default;

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    const KinematicsVector<Id, KinematicsValue>&) = default;

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>&
KinematicsVector<Id, KinematicsValue>::operator=(
    KinematicsVector<Id, KinematicsValue>&&) = default;

template <typename Id, typename KinematicsValue>
KinematicsVector<Id, KinematicsValue>::~KinematicsVector() = default;

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::clear() {
  /* We don't use map.clear() to ensure the memory isn't released. */
  pimpl_->map.erase(pimpl_->map.begin(), pimpl_->map.end());
}

template <typename Id, typename KinematicsValue>
void KinematicsVector<Id, KinematicsValue>::set_value(
    Id id, const KinematicsValue& value) {
  pimpl_->map.insert_or_assign(id, value);
}

template <typename Id, typename KinematicsValue>
int KinematicsVector<Id, KinematicsValue>::size() const {
  return pimpl_->map.size();
}

namespace {
// This is a separate function in order to minimize the size of the object code
// in `value()` when on the hot path, and so that the exception-throwing object
// code (which is quite large) will be shared across all of the scalar types.
template <typename Id>
[[noreturn]] void ThrowNoSuchId(Id id) {
  throw std::runtime_error(fmt::format(
      "No such {}: {}.",
      NiceTypeName::RemoveNamespaces(NiceTypeName::Get<Id>()), to_string(id)));
}
}  // namespace

template <typename Id, typename KinematicsValue>
const KinematicsValue& KinematicsVector<Id, KinematicsValue>::value(
    Id id) const {
  const auto& it = pimpl_->map.find(id);
  if (it != pimpl_->map.end()) {
    return it->second;
  }
  ThrowNoSuchId(id);
}

template <typename Id, typename KinematicsValue>
bool KinematicsVector<Id, KinematicsValue>::has_id(Id id) const {
  return pimpl_->map.contains(id);
}

template <typename Id, typename KinematicsValue>
std::vector<Id> KinematicsVector<Id, KinematicsValue>::frame_ids() const {
  return GetAllIds();
}

template <typename Id, typename KinematicsValue>
std::vector<Id> KinematicsVector<Id, KinematicsValue>::GetAllIds() const {
  std::vector<Id> results(pimpl_->map.size());
  int index = 0;
  for (const auto& it : pimpl_->map) {
    results[index++] = it.first;
  }
  // Sort the results to ensure it's deterministic.
  std::sort(results.begin(), results.end());
  return results;
}

// Explicitly instantiates on the most common scalar types.
using math::RigidTransform;
using symbolic::Expression;
template class KinematicsVector<FrameId, RigidTransform<double>>;
template class KinematicsVector<FrameId, RigidTransform<AutoDiffXd>>;
template class KinematicsVector<FrameId, RigidTransform<Expression>>;
template class KinematicsVector<GeometryId, VectorX<double>>;
template class KinematicsVector<GeometryId, VectorX<AutoDiffXd>>;
template class KinematicsVector<GeometryId, VectorX<Expression>>;

}  // namespace geometry
}  // namespace drake
