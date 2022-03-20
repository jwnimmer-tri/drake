#pragma once

#include <mutex>
#include <thread>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/unused.h"

namespace drake {
namespace internal {

/** Manages a pool of thread-local storage to a contained type T. Each thread
that accesses the contained element will receive a uniquely-owned reference,
keyed on its thread_id. The contained elements are created lazily (i.e., on
demand). The contained elements are destroyed when this object is destroyed
(i.e., not upon thread termination). */
template <typename T>
class thread_local_ptr final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(thread_local_ptr);

  /** Creates an empty container. */
  thread_local_ptr() = default;

  /** Returns the thread-local reference to our contained T. If there was no
  contained instance yet, then default-constructs one and returns it. */
  T& operator*() {
    // TODO(jwnimmer-tri) Reduce mutex contention to speed this up.
    std::lock_guard guard(mutex_);
    // TODO(jwnimmer-tri) Might need to key on OMP thread ID here as well,
    // iff we're within an OMP parallel section.
    auto [iter, created] = data_.try_emplace(std::this_thread::get_id());
    std::unique_ptr<T>& result = iter->second;
    if (created) {
      result = std::make_unique<T>();
    }
    DRAKE_ASSERT(result != nullptr);
    return *result;
  }

 private:
  std::mutex mutex_;
  std::unordered_map<std::thread::id, std::unique_ptr<T>> data_;
};

}  // namespace internal
}  // namespace drake
