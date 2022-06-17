#pragma once

#include <type_traits>
#include <utility>

namespace drake {
namespace internal {

/* FastPimpl<class Impl> is used in place of std::unique_ptr<class Impl> to
obtain inline storage (i.e., no extra pointer chasing) while still retaining
the #include graph firewall benefits of the Pimpl design pattern.

@tparam T the forward-declared Impl class or structT
@tparam pimpl_size must be >= sizeof(T)
@tparam pimpl_alignment must be == N * alignof(T) where N is either 1, 2, or 4.
*/
template <typename T, std::size_t pimpl_size, std::size_t pimpl_alignment>
class FastPimpl {
 public:
  /* Delegating constructor for T(Args...). */
  template <typename... Args, typename = std::enable_if_t<
      !(sizeof...(Args) == 1 && std::conjunction_v<std::is_same<T, Args>...>)>>
  explicit FastPimpl(Args&&... args) noexcept(
      std::is_nothrow_constructible_v<T, Args&&...>) {
    validate<sizeof(T), alignof(T)>();
    new (&data_) T(std::forward<Args>(args)...);
  }

  /* Copy constructor. */
  FastPimpl(const FastPimpl& other) noexcept(
      std::is_nothrow_copy_constructible_v<T>) {
    new (&data_) T(*other);
  }

  /* Move constructor. */
  FastPimpl(FastPimpl&& other) noexcept(
      std::is_nothrow_move_constructible_v<T>) {
    new (&data_) T(std::move(*other));
  }

  /* Copy assignment. */
  FastPimpl& operator=(const FastPimpl& rhs) noexcept(
      std::is_nothrow_copy_assignable_v<T>) {
    **this = *rhs;
    return *this;
  }

  /* Move assignment. */
  FastPimpl& operator=(FastPimpl&& rhs) noexcept(
      std::is_nothrow_move_assignable_v<T>) {
    **this = std::move(*rhs);
    return *this;
  }

  /* Destructor. */
  ~FastPimpl() noexcept { (**this).~T(); }

  /* Accessors (mutable). */
  T* operator->() noexcept {
    return reinterpret_cast<T*>(&data_);
  }
  T& operator*() noexcept {
    return reinterpret_cast<T&>(data_);
  }

  /* Accessors (readonly). */
  const T* operator->() const noexcept {
    return reinterpret_cast<const T*>(&data_);
  }
  const T& operator*() const noexcept {
    return reinterpret_cast<const T&>(data_);
  }

 private:
  // Checks that the class-template size and alignment are suitable for T.
  // In order to support forward-declared Impl classes, we need to defer this
  // checking to construction-time (not, e.g., a static_assert at class scope).
  template <std::size_t actual_size, std::size_t actual_alignment>
  static void validate() noexcept {
    // If these assertions fail, the compiler's error message will mention the
    // function template arguments which will tell you T's actual sizes.
    static_assert(pimpl_size >= actual_size, "sizeof(T) mismatch");
    static_assert(
        (pimpl_alignment == actual_alignment) ||
        (pimpl_alignment == actual_alignment * 2) ||
        (pimpl_alignment == actual_alignment * 4),
        "alignof(T) mismatch");
  }

  std::aligned_storage_t<pimpl_size, pimpl_alignment> data_;
};

}  // namespace internal
}  // namespace drake
