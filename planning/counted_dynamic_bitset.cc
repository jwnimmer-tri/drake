#include "drake/planning/counted_dynamic_bitset.h"

#include <algorithm>
#include <cstring>
#include <utility>

namespace drake {
namespace planning {
namespace internal {

namespace {

// Our inline storage can store this many bits.
constexpr int kMaxInlineSize = 128;

// Returns the number of 1 bits in `x`.
template <typename SomeUInt>
int popcount(SomeUInt x) {
  return __builtin_popcount(x);
}

// Copies a span of bools into a new unique_ptr<bool[]>.
// @pre values.size() > kMaxInlineSize
__attribute__((noinline, cold)) std::unique_ptr<bool[]> MakeBoolArray(
    std::span<const bool> values) {
  DRAKE_ASSERT(values.size() > kMaxInlineSize);
  if (!(values.size() > kMaxInlineSize)) __builtin_unreachable();
  // TODO(jwnimmer-tri) Use make_unique_for_overwrite once we have >= C++23.
  std::unique_ptr<bool[]> result(new bool[values.size()]);
  std::copy(values.begin(), values.end(), result.get());
  return result;
}

// Broadcasts a single bool into a new unique_ptr<bool[]>.
// @pre size > kMaxInlineSize
__attribute__((noinline, cold)) std::unique_ptr<bool[]> MakeBoolArray(
    int size, bool value = false) {
  DRAKE_ASSERT(size > kMaxInlineSize);
  if (!(size > kMaxInlineSize)) __builtin_unreachable();
  // TODO(jwnimmer-tri) Use make_unique_for_overwrite once we have >= C++23.
  std::unique_ptr<bool[]> result(new bool[size]);
  std::fill_n(result.get(), size, value);
  return result;
}

// Copies the bool array at `other` into its complement (negation).
// @pre size > kMaxInlineSize
__attribute__((noinline, cold)) std::unique_ptr<bool[]> MakeComplement(
    std::span<const bool> other) {
  DRAKE_ASSERT(other.size() > kMaxInlineSize);
  if (!(other.size() > kMaxInlineSize)) __builtin_unreachable();
  // TODO(jwnimmer-tri) Use make_unique_for_overwrite once we have >= C++23.
  std::unique_ptr<bool[]> result(new bool[other.size()]);
  for (size_t i = 0; i < other.size(); ++i) {
    result[i] = !other[i];
  }
  return result;
}

// The template argument for Merge(), immediately below.
enum BinaryOperator {
  kUnion,
  kIntersect,
  kSubtract,
};

// Returns a new bool array computed as a binary operator over the equally-
// sized `a` and `b` input arrays.
// @pre size > kMaxInlineSize
template <BinaryOperator operation>
__attribute__((noinline,
               cold)) std::pair<std::unique_ptr<bool[]>, int /* count */>
Merge(int size, const bool* a, const bool* b) {
  DRAKE_ASSERT(size > kMaxInlineSize);
  if (!(size > kMaxInlineSize)) __builtin_unreachable();
  // TODO(jwnimmer-tri) Use make_unique_for_overwrite once we have >= C++23.
  std::unique_ptr<bool[]> buffer(new bool[size]);
  int count = 0;
  for (int i = 0; i < size; ++i) {
    int bit;
    if constexpr (operation == kUnion) {
      bit = int{a[i]} | int{b[i]};
    } else if constexpr (operation == kIntersect) {
      bit = int{a[i]} & int{b[i]};
    } else if constexpr (operation == kSubtract) {
      bit = int{a[i]} & ~int{b[i]};
    } else {
      DRAKE_UNREACHABLE();
    }
    buffer[i] = (bit != 0);
    count += bit;
  }
  return std::make_pair(std::move(buffer), count);
}

}  // namespace

CountedDynamicBitset::CountedDynamicBitset(int size, bool value) {
  DRAKE_THROW_UNLESS(size >= 0);
  size_ = size;
  count_ = value ? size : 0;
  if (size <= kMaxInlineSize) [[likely]] {
    if (value) {
      inline_ = mask();
    }
  } else {
    buffer_ = MakeBoolArray(size, value);
  }
  CheckInvariants();
}

CountedDynamicBitset::CountedDynamicBitset(std::span<const bool> values) {
  static_assert(kMaxInlineSize == sizeof(SmallBitmaskUInt) * 8);
  size_ = ssize(values);
  if (size_ <= kMaxInlineSize) [[likely]] {
    for (int i = 0; i < size_; ++i) {
      const bool bit = values[i];
      inline_ |= (SmallBitmaskUInt{bit} << i);
    }
    count_ = popcount(inline_);
  } else {
    buffer_ = MakeBoolArray(values);
    count_ = std::count(values.begin(), values.end(), true);
  }
  CheckInvariants();
}

// We can't use the defaulted implementation because of our unique_ptr member.
// However, for simplicity we can just delegate to the copy-assignment operator.
CountedDynamicBitset::CountedDynamicBitset(const CountedDynamicBitset& other) {
  *this = other;
  CheckInvariants();
}

// We can't use the defaulted implementation because of our unique_ptr member.
CountedDynamicBitset& CountedDynamicBitset::operator=(
    const CountedDynamicBitset& other) {
  if (this == &other) [[unlikely]] {
    return *this;
  }
  size_ = other.size();
  count_ = other.count();
  inline_ = other.inline_;
  if (other.is_inline()) [[likely]] {
    buffer_.reset();
  } else {
    buffer_ =
        MakeBoolArray(std::span<const bool>(other.buffer_.get(), other.size_));
  }
  CheckInvariants();
  return *this;
}

CountedDynamicBitset CountedDynamicBitset::Complement() const {
  if (is_inline()) [[likely]] {
    return CountedDynamicBitset(
        /* size = */ size(),
        /* count = */ size() - count(),
        /* inline = */ ~inline_ & mask());
  } else {
    CountedDynamicBitset result(
        /* size = */ size(),
        /* count = */ size() - count(),
        /* inline = */ 0);
    result.buffer_ =
        MakeComplement(std::span<const bool>(buffer_.get(), size()));
    result.CheckInvariants();
    return result;
  }
}

CountedDynamicBitset CountedDynamicBitset::Union(
    const CountedDynamicBitset& other) const {
  DRAKE_ASSERT(this->size() == other.size());
  DRAKE_ASSERT(this->is_inline() == other.is_inline());
  if (is_inline()) [[likely]] {
    const SmallBitmaskUInt new_bits = (this->inline_ | other.inline_) & mask();
    return CountedDynamicBitset(size(),
                                /* count = */ popcount(new_bits),
                                /* inline = */ new_bits);
  } else {
    CountedDynamicBitset result(size(), /* count = */ 0, /* inline = */ 0);
    std::tie(result.buffer_, result.count_) =
        Merge<kUnion>(size(), buffer_.get(), other.buffer_.get());
    result.CheckInvariants();
    return result;
  }
}

CountedDynamicBitset CountedDynamicBitset::Intersect(
    const CountedDynamicBitset& other) const {
  DRAKE_ASSERT(this->size() == other.size());
  DRAKE_ASSERT(this->is_inline() == other.is_inline());
  if (is_inline()) [[likely]] {
    const SmallBitmaskUInt new_bits = (this->inline_ & other.inline_) & mask();
    return CountedDynamicBitset(size(),
                                /* count = */ popcount(new_bits),
                                /* inline = */ new_bits);
  } else {
    CountedDynamicBitset result(size(), /* count = */ 0, /* inline = */ 0);
    std::tie(result.buffer_, result.count_) =
        Merge<kIntersect>(size(), buffer_.get(), other.buffer_.get());
    result.CheckInvariants();
    return result;
  }
}

CountedDynamicBitset CountedDynamicBitset::Subtract(
    const CountedDynamicBitset& other) const {
  DRAKE_ASSERT(this->size() == other.size());
  DRAKE_ASSERT(this->is_inline() == other.is_inline());
  if (is_inline()) [[likely]] {
    const SmallBitmaskUInt new_bits = (this->inline_ & ~other.inline_) & mask();
    return CountedDynamicBitset(size(),
                                /* count = */ popcount(new_bits),
                                /* inline = */ new_bits);
  } else {
    CountedDynamicBitset result(size(), /* count = */ 0, /* inline = */ 0);
    std::tie(result.buffer_, result.count_) =
        Merge<kSubtract>(size(), buffer_.get(), other.buffer_.get());
    result.CheckInvariants();
    return result;
  }
}

bool CountedDynamicBitset::operator==(const CountedDynamicBitset& other) const {
  if (this->size() != other.size()) {
    return false;
  }
  DRAKE_ASSERT(this->is_inline() == other.is_inline());
  bool result;
  if (is_inline()) [[likely]] {
    result = (this->inline_ == other.inline_);
  } else {
    result = (std::memcmp(buffer_.get(), other.buffer_.get(), size()) == 0);
  }
  DRAKE_ASSERT((result == false) || (this->count() == other.count()));
  return result;
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
