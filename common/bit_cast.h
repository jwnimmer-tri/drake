#pragma once

#ifdef __cpp_lib_bit_cast

#include <bit>

namespace drake {
namespace internal {

using std::bit_cast;

}  // namespace internal
}  // namespace drake

#else  // __cpp_lib_bit_cast

#include <cstring>
#include <type_traits>

namespace drake {
namespace internal {

/** Implements C++ https://en.cppreference.com/w/cpp/numeric/bit_cast (but
without the overload resolution guards, which are not necessary in our case.)
(Once Drake requires GCC >= 11, i.e., once we drop support for
Ubuntu 20.04 Focal, then we can remove this function in lieu of the std one.) */
template <class To, class From>
To bit_cast(const From& from) noexcept {
  static_assert(std::is_trivially_constructible_v<To>);
  To result;
  static_assert(sizeof(To) == sizeof(From));
  static_assert(std::is_trivially_copyable_v<To>);
  static_assert(std::is_trivially_copyable_v<From>);
  std::memcpy(&result, &from, sizeof(result));
  return result;
}

}  // namespace internal
}  // namespace drake

#endif  // __cpp_lib_bit_cast
