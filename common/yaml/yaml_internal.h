#pragma once

#include <optional>
#include <tuple>
#include <utility>
#include <variant>

namespace drake {
namespace yaml {
namespace internal {

/* A type-helper struct to cull std::monostate from the `Types...`. */
template <template <typename...> class Variant, typename... Types>
struct VariantSansMonostate {
  /* A std::tuple<Types...> but without the monostate type. */
  using TupleSansMonostate = decltype(std::tuple_cat(
      std::declval<
          typename std::conditional_t<std::is_same_v<Types, std::monostate>,
                                      std::tuple<>, std::tuple<Types>>>()...));

  template <typename... Ts>
  static auto MakeVariantFromTuple(const std::tuple<Ts...>&) {
    return Variant<Ts...>{};
  }

  using type =
      decltype(MakeVariantFromTuple(std::declval<TupleSansMonostate>()));
};

/* Given an `input` value that allows `std::monostate`, returns a copy but where
the `monostate` has been replaced by an `std::optional` wrapper, e.g., copies a
`variant<monostate, int, string>` to an `optional<variant<int, string>>`. */
template <template <typename...> class Variant, typename... Types>
auto CopyMonostateVariantToOptionalVariant(const Variant<Types...>& input) {
  std::optional<typename VariantSansMonostate<Variant, Types...>::type> result;
  std::visit(
      [&result](const auto& x) {
        if constexpr (!std::is_same_v<decltype(x), const std::monostate&>) {
          result = x;
        }
      },
      input);
  return result;
}

}  // namespace internal
}  // namespace yaml
}  // namespace drake
