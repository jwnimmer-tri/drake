#include "drake/common/ad/internal/partials.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/ad/internal/static_unit_vector.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::VectorXd;

// Narrow an Eigen::Index to a plain int index, throwing when out-of-range.
int IndexToInt(Eigen::Index index) {
  if (index < 0) {
    throw std::out_of_range(fmt::format(
        "AutoDiff derivatives size or offset {} is negative", index));
  }
  if (index > INT32_MAX) {
    throw std::out_of_range(fmt::format(
        "AutoDiff derivatives size or offset {} is too large", index));
  }
  return static_cast<int>(index);
}

}  // namespace

StorageVec StorageVec::Allocate(int size) {
  DRAKE_ASSERT(0 <= size && size <= INT32_MAX);
  StorageVec result;
  if (size > 0) {
    result.size_ = size;
    if (size == 1) {
      result.unit_ = 1;
    } else {
      result.data_ = new double[size];
    }
  }
  return result;
}

StorageVec::StorageVec(const StorageVec& other) noexcept {
  DRAKE_ASSERT(0 <= other.size_ && other.size_ <= INT32_MAX);
  StorageVec result;
  if (other.size_ > 0) {
    size_ = other.size_;
    data_ = new double[size_];
    std::copy(other.data_, other.data_ + size_, data_);
  }
}

StorageVec& StorageVec::operator=(const StorageVec& other) noexcept {
  DRAKE_ASSERT(0 <= other.size_ && other.size_ <= INT32_MAX);
  if (this != &other) {
    if (size_ == other.size_) {
      std::copy(other.data_, other.data_ + other.size_, data_);
    } else {
      delete data_;
      size_ = other.size_;
      if (size_ > 0) {
        data_ = new double[size_];
        std::copy(other.data_, other.data_ + other.size_, data_);
      } else {
        data_ = nullptr;
      }
    }
  }
  return *this;
}

StorageVec::~StorageVec() {
  delete data_;
}

Partials::Partials(Eigen::Index size, Eigen::Index offset)
    : coeff_{1.0}, storage_{IndexToInt(size), IndexToInt(offset)} {
  if (IndexToInt(offset) >= size) {
    throw std::out_of_range(fmt::format(
        "AutoDiff offset {} must be strictly less than size {}", offset, size));
  }
  // For really big unit derivatives, we can't use GetStaticUnitVector() for
  // the make_const_xpr() readback, so they'll need to live on the heap. If
  // this ends up being a problem, we can increase kMaxStaticVectorSize or
  // add a compressed (i.e., sparse) storage option to CowVec.
  if (size > kMaxStaticVectorSize) {
    *this = Partials{VectorXd::Unit(size, offset)};
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
}

Partials::Partials(const Eigen::Ref<const VectorXd>& value)
    : coeff_{1.0}, storage_{} {
  const int size = IndexToInt(value.size());
  if (size == 0) {
    // Nothing else to do.
  } else if (size == 1) {
    coeff_ = value[0];
    storage_ = StorageVec(1, 0);
  } else {
    storage_ = StorageVec::Allocate(size);
    mutable_storage_view() = value;
  }
  DRAKE_ASSERT_VOID(CheckInvariants());
}

// Representation invariant:
// - size >= 0
// - if size == 0:
//   - storage.data == null
// - if size == 1:
//   - 0 < unit <= size
//   - storage.data == null
// - if size >= 2:
//   - 0 <= unit <= size
//   - coeff != 0.0
//   - if unit > 0:
//     - storage.data == null
//   - if unit == 0:
//     - storage.data != null
//     - storage.data contains storage for exactly size elements
void Partials::CheckInvariants() const {
  if (storage_.size_ == 1) {
    // When we denote exactly one partial, it must always be stored inline.
    DRAKE_DEMAND(unit_ == 1);
    DRAKE_DEMAND(storage_.data() == nullptr);
  } else {
    // This is the general case (size >= 2).
    DRAKE_DEMAND(unit_ >= 0);
    DRAKE_DEMAND(unit_ <= size_);
    DRAKE_DEMAND(coeff_ != 0.0);
    if (unit_ > 0) {
      // When we denote exactly one partial, it must always be stored inline.
      DRAKE_DEMAND(storage_.data() == nullptr);
    } else {
      // When we denote >1 partials, we must have allocated heap for them.
      DRAKE_DEMAND(storage_.data() != nullptr);
      DRAKE_DEMAND(storage_.get_use_count() >= 1);
    }
  }
}

void Partials::MatchSizeOf(const Partials& other) {
  // Partials with size() == 0 may be freely combined with any other size.
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    // XXX can/should we use a unit vector here?
    storage_ = StorageVec::Allocate(other.size());
    mutable_storage_view().setZero();
    coeff_ = 1.0;
    return;
  }
  ThrowIfDifferentSize(other);
}

void Partials::SetZero() {
  storage_ = StorageVec(size(), 0);
  coeff_ = 0.0;
  DRAKE_ASSERT_VOID(CheckInvariants());
}

void Partials::Mul(double factor) {
  if (std::isfinite(factor)) [[likely]] {
    // The `coeff_` will remain finite; multiplying two finite numbers always
    // produces a finite result.
    coeff_ *= factor;
  } else {
    // Do a "sparse" multiply of only the non-zero values in storage. If the
    // value is zero or NaN, then we'll leave it alone.
    const double total_factor = coeff_ * factor;  // This will be non-finite.
    ad::DerivativesMutableXpr partials = MakeMutableXpr();
    for (int i = 0; i < size(); ++i) {
      const double x = partials[i];
      if ((x < 0) || (x > 0)) {
        partials[i] = x * total_factor;
      }
    }
    DRAKE_ASSERT_VOID(CheckInvariants());
  }
}

void Partials::Div(double factor) {
  if (!std::isnan(factor) && factor != 0.0) [[likely]] {
    // The `coeff_` will remain finite; dividing finite by ±∞ produces ±0.
    coeff_ /= factor;
  } else {
    // Do a "sparse" division of only the non-zero values in storage. If the
    // value is zero or NaN, then we'll leave it alone.
    const double total_factor = coeff_ / factor;
    ad::DerivativesMutableXpr partials = MakeMutableXpr();
    for (int i = 0; i < size(); ++i) {
      const double x = partials[i];
      if ((x < 0) || (x > 0)) {
        partials[i] = x * total_factor;
      }
    }
    DRAKE_ASSERT_VOID(CheckInvariants());
  }
}

void Partials::Add(const Partials& other) {
  AddScaled(1.0, other);
}

void Partials::AddScaled(double scale, const Partials& other) {
  DRAKE_ASSERT_VOID(CheckInvariants());
  DRAKE_ASSERT_VOID(other.CheckInvariants());

  // Handle the case of this and/or other being empty (treated as all-zero).
  // Partials with size() == 0 may be freely combined with any other size.
  if (other.size() == 0) {
    return;
  }
  if (size() == 0) {
    *this = other;
    Mul(scale);
    return;
  }
  ThrowIfDifferentSize(other);

  // Unlikely: non-finite scale; zero partials in `other` must not be scaled.
  if (!std::isfinite(scale)) [[unlikely]] {
    ad::DerivativesMutableXpr this_data = MakeMutableXpr();
    const ad::DerivativesConstXpr other_data = other.make_const_xpr();
    for (int i = 0; i < size; ++i) {
      const double this_datum = this_data[i];
      const double other_datum = other_data[i];
      if ((other_datum < 0) || (other_datum > 0)) {
        this_data[i] = this_datum + other_datum * scale;
      }
    }
  }

  // The implementation that follows has many special cases for performance.
  //
  // The easiest way to follow along is to recognize that the case-analysis
  // branches first on the representation of `this` from lowest complexity
  // to highest complexity:
  //   Case 1 -- `this` is a scaled unit vector.
  //   Case 2 -- `this` is a scaled uniquely-owned heap vector.
  //   Case 3 -- `this` is a scaled shared-ownership heap vector.
  //
  // Then secondarily, we branch on the nature of `other`.

  // === Case 1 -- this is a scaled unit vector. ===

  if (is_unit()) {
    // If `other` is the same unit vector, the addition is cheap.
    if (unit_ == other.unit_) {
      coeff_ += scale * other.coeff_;
      if (coeff_ == 0.0) {
        SetZero();
      }
      return;
    }

    // Otherwise, we need to allocate storage for a non-unit vector.
    storage_ = CowVec::Allocate(size);
    double* new_data = storage_.mutable_data();

    // If this and other were both unit vectors, just set the two partials.
    if (other.is_unit()) {
      for (int i = 0; i < size; ++i) {
        new_data[i] = 0.0;
      }
      new_data[get_unit_index()] = coeff_;
      new_data[other.get_unit_index()] = scale * other.coeff_;
      coeff_ = 1.0;
      unit_ = 0;
      DRAKE_ASSERT_VOID(CheckInvariants());
      return;
    }

    // The `other` was a full vector, so we'll need to copy its data and then
    // add our unit. We'll inherit other's coeff so that we don't need to
    // multiply it through each vector element.
    const double new_coeff = scale * other.coeff_;
    const double this_relative_partial = coeff_ / new_coeff;
    const double* const other_data = other.storage_.data();
    for (int i = 0; i < size; ++i) {
      new_data[i] = other_data[i];
    }
    new_data[get_unit_index()] += this_relative_partial;
    coeff_ = new_coeff;
    unit_ = 0;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }

  // === Case 2 -- `this` is a scaled uniquely-owned heap vector. ===

  // If we have exclusive storage, then we can mutate it in place.
  if (storage_.get_use_count() == 1) {
    double* this_data = storage_.mutable_data();

    // If `other` is a unit vector, the addition is cheap.
    if (other.is_unit()) {
      this_data[other.get_unit_index()] += (scale * other.coeff_) / coeff_;
      return;
    }

    // Otherwise, we'll need to fold in the other's data.
    // We use a linear combination and end up with a new coeff of 1.0.
    const double* const other_data = other.storage_.data();
    for (int i = 0; i < size; ++i) {
      this_data[i] =
          coeff_ * this_data[i] + (scale * other.coeff_) * other_data[i];
    }
    coeff_ = 1.0;
    unit_ = 0;
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }

  // === Case 3 -- `this` is a scaled shared-ownership heap vector. ===

  // It's not uncommon to multiply (x + x') by itself, which in autodiff looks
  // like x² + x*x' + x*x'. Recognizing that x' is the same across both addends
  // save us a copy.
  if (storage_.data() == other.storage_.data()) {
    DRAKE_DEMAND(!other.is_unit());
    coeff_ += scale * other.coeff_;
    if (coeff_ == 0.0) {
      SetZero();
    }
    return;
  }

  // We've run out of tricks; we have no choice now but to allocate fresh
  // storage to accumulate the result.
  CowVec new_storage = CowVec::Allocate(size);
  double* new_this_data = new_storage.mutable_data();
  const double* const original_this_data = storage_.data();

  // If `other` was a unit vector, we can copy our data and add the single unit.
  if (other.is_unit()) {
    for (int i = 0; i < size; ++i) {
      new_this_data[i] = original_this_data[i];
    }
    new_this_data[other.get_unit_index()] += scale * other.coeff_ / coeff_;
    storage_ = std::move(new_storage);
    DRAKE_ASSERT_VOID(CheckInvariants());
    return;
  }

  // We've reached the most general case. Both partials are distinct, readonly
  // heap vectors.
  Eigen::Map<VectorXd> output_vector(new_this_data, size);
  output_vector.noalias() =
      coeff_ * Eigen::Map<const VectorXd>(storage_.data(), size) +
      (scale * other.coeff_) *
          Eigen::Map<const VectorXd>(other.storage_.data(), size);
  storage_ = std::move(new_storage);
  coeff_ = 1.0;
  DRAKE_ASSERT_VOID(CheckInvariants());
}

DerivativesConstXpr Partials::make_const_xpr() const {
  const double* data = nullptr;
  if (is_unit()) {
    data = GetStaticUnitVector(get_unit_index());
  } else {
    data = storage_.data();
  }
  return DerivativesConstXpr{coeff_, data, size(), /* stride = */ 1};
}

DerivativesMutableXpr Partials::MakeMutableXpr() {
  double* data = nullptr;
  if (size() == 0) {
    // Nothing to do.
  } else if (size() == 1) {
    DRAKE_ASSERT(is_unit());
    // We'll just store it directly in `coeff_`.
    data = &coeff_;
  } else {
    data = storage_.mutable_data();
    DRAKE_ASSERT(data != nullptr);
  }
  return DerivativesMutableXpr{this, data, size()};
}

void Partials::ThrowIfDifferentSize(const Partials& other) {
  // If this check trips, then that means the user tried to mix AutoDiff partial
  // derivatives of non-uniform sizes.
  if (size() != other.size()) {
    throw std::logic_error(fmt::format(
        "The size of AutoDiff partial derivative vectors must be uniform"
        " throughout the computation, but two different sizes ({} and {})"
        " were encountered at runtime.",
        size(), other.size()));
  }
}

DerivativesMutableXpr Partials::SetFrom(
    const Eigen::Ref<const VectorXd>& other) {
  *this = Partials(other);
  return MakeMutableXpr();
}

// We want sizeof(AutoDiff) to be <= 32 bytes. Setting aside the 8 bytes for
// the value, that leaves 24 bytes available for the Partials.
static_assert(sizeof(Partials) <= 24);

}  // namespace internal
}  // namespace ad
}  // namespace drake
