#pragma once

#include <cstdlib>

#include "drake/common/ad/internal/derivatives_xpr.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace ad {
namespace internal {

/* Heap storage for an array of doubles, for use by the Partials class later in
this file. The storage can be empty (null).

Note that this class is not directly unit tested; instead, it's test coverage
comes from partial_test indirectly. */
class StorageVec {
 public:
  /* Allocates new storage of the given size, but does not initialize it.
  If the size is zero, the storage will be empty (null). */
  static StorageVec Allocate(int size);

  /* Constructs empty (null) storage. */
  StorageVec() = default;

  /* Constructs unit vector storage. */
  StorageVec(int size, int offset)
      : size_(size), unit_(offset + 1), data_(nullptr) {}

  /* Steals the storage from `other`. */
  StorageVec(StorageVec&& other) noexcept {
    size_ = other.size_;
    unit_ = other.unit_;
    data_ = other.data_;
    other.size_ = 0;
    other.unit_ = 0;
    other.data_ = nullptr;
  }

  /* Steals the storage from `other`. */
  StorageVec& operator=(StorageVec&& other) noexcept {
    if (this != &other) {
      delete data_;
      size_ = other.size_;
      unit_ = other.unit_;
      data_ = other.data_;
      other.size_ = 0;
      other.unit_ = 0;
      other.data_ = nullptr;
    }
    return *this;
  }

  /* Copies the storage from `other`. */
  StorageVec(const StorageVec& other) noexcept;

  /* Copies the storage from `other`. */
  StorageVec& operator=(const StorageVec& other) noexcept;

  ~StorageVec();

  /* Returns the size. */
  int size() const { return size_; }

  // XXX deal with unit vectors
  /* Returns the double array storage (or null, when empty). */
  const double* data() const { return data_; }

  /* Returns the mutable double array storage.
  @pre size() > 1 */
  double* mutable_data() {
    DRAKE_ASSERT(size _ > 1);
    return data_;
  }

 private:
  int32_t size_{0};
  int32_t unit_{0};
  double* data_{nullptr};
};

/* A vector of partial derivatives, optimized for use with Drake's AutoDiff.

Partials are dynamically sized, and can have size() == 0.

When adding two Partials, they must have a compatible size(). A Partials with
size() == 0 may be freely combined with any other size, and is treated as-if
it were filled with all zeros. When adding two Partials where each one has a
non-zero size, the two sizes must be identical.

Note that a Partials object with non-zero size might still be filled with zeros
for its values. Once a non-zero size() has been established, it is "sticky" and
never returns back to being zero-sized (unless the Partials is moved-from).

In particular, note that the result of a binary operation takes on the size from
either operand, e.g., foo.Add(bar) with foo.size() == 0 and bar.size() == 4 will
will result in foo.size() == 4 after the addition, and that's true even if bar's
vector was all zeros.

When a scale factor is applied to a Partials object (e.g., with Mul, Div, or
AddScaled), any zero values will remain zero, even if the factor is ±∞ or NaN.
We treat them as "missing" (i.e., sparse), not IEEE zero, so multiplication by
non-finite numbers is still well-defined.

A large portion of this class definition appears inline, because AutoDiff is
used heavily in inner loops. The general approach is that simple word-sized
sets, gets, and branches appear inline; functions that do more than a couple
branches or touch more than a couple words (and all functions that allocate)
are out-of-line (in the cc file). */
class Partials {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Partials);

  /* Constructs an empty vector. */
  Partials() = default;

  /* Constructs a single partial derivative of `coeff` at the given `offset` in
  a vector of `size` otherwise-zero derivatives.
  @throws std::exception if offset >= size */
  Partials(Eigen::Index size, Eigen::Index offset);

  /* Constructs a vector with a copy of the given value. */
  explicit Partials(const Eigen::Ref<const Eigen::VectorXd>& value);

  ~Partials() = default;

  /* Returns the size of this vector. */
  int size() const { return storage_.size(); }

  /* Updates `this` to be the same size as `other`.
  If `this` and `other` are already the same size then does nothing.
  Otherwise, if `other` has size 0 then does nothing.
  Otherwise, if `this` has size 0 then sets this to a zero vector of size
  `other.size`.
  Otherwise, throws an exception for mismatched sizes. */
  void MatchSizeOf(const Partials& other);

  /* Set this to zero. */
  void SetZero();

  /* Scales this vector by the given amount. */
  void Mul(double factor);

  /* Scales this vector by the reciprocal of the given amount. */
  void Div(double factor);

  /* Adds `other` into `this`. */
  void Add(const Partials& other);

  /* Adds `scale * other` into `this`. */
  void AddScaled(double scale, const Partials& other);

  /* Returns an Eigen-compatible view into this vector. */
  ad::DerivativesConstXpr make_const_xpr() const;

  /* Returns an Eigen-compatible mutable view into this vector, including
  resizing. This runs in linear time O(size). */
  ad::DerivativesMutableXpr MakeMutableXpr();

 private:
  void ThrowIfDifferentSize(const Partials& other);

  /* Returns a const Eigen view of the current storage (i.e., without scaling
  by the `coeff_`). */
  Eigen::Map<const Eigen::VectorXd> storage_view() const {
    return Eigen::Map<const Eigen::VectorXd>(storage_.data(), storage_.size());
  }

  /* Returns a mutable Eigen view of the current storage (i.e., without scaling
  by the `coeff_`). */
  Eigen::Map<Eigen::VectorXd> mutable_storage_view() {
    return Eigen::Map<Eigen::VectorXd>(storage_.mutable_data(),
                                       storage_.size());
  }

  /* Returns true iff this represents a (scaled) unit vector. */
  bool is_unit() const { return unit_ > 0; }

  /* Returns the index of the non-zero element of this (scaled) unit vector.
  @pre is_unit() */
  int get_unit_index() const {
    DRAKE_ASSERT(unit_ > 0);
    return unit_ - 1;
  }

  // Our MutableXpr type is allowed to set us via a backreference.
  friend ad::DerivativesMutableXpr;
  ad::DerivativesMutableXpr SetFrom(
      const Eigen::Ref<const Eigen::VectorXd>& other);

  void CheckInvariants() const;

  // XXX update for unit vector
  // Abstraction function:
  // - When `size == 0`, the partials vector is empty (zero everywhere),
  //   and may be combined with any other vector (i.e., it has no defined size).
  // - When `size > 0`:
  //   - The partials vector has a size of `size`.
  //   - When `unit > 0`, the partials are zero everywhere except
  //     for the `unit-1`th partial where it has the value of `coeff`.
  //   - When `unit == 0` the partials value is `coeff * storage`
  //     where the storage is a full-size dense vector on the heap.
  //
  //
  // Our effective value is `coeff_ * storage_`; we store them separately so
  // that re-scaling is fast (we can just scale the coeff).
  //
  // We maintain an invariant that `coeff_` is always finite. If a modification
  // to it (e.g., multiplication by a factor) would cause it to become non-
  // finite, then instead of multiplying the factor into `coeff_`, instead we
  // "sparse" multiply it through the `storage_` (i.e., only multiplying it
  // through the non-zero terms in `storage_`). This is required to meet our API
  // contract of "any zero values will remain zero, even if the factor is ±∞ or
  // NaN" per our class overview.
  double coeff_{0.0};
  StorageVec storage_;
};

}  // namespace internal
}  // namespace ad
}  // namespace drake
