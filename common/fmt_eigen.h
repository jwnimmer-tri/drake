#pragma once

#include <iterator>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>

#include "drake/common/fmt.h"

namespace drake {
namespace internal {

/* A tag type to be used in fmt::format("{}", fmt_eigen(...)) calls.
Below we'll add a fmt::formatter<> specialization for this tag. */
template <typename Derived>
struct fmt_eigen_ref {
  const Eigen::MatrixBase<Derived>& matrix;
};

/* Given the already string-formatted elements of a (rows x cols)-sized matrix,
layout the elements with brackets and commas. The elements are stored in the
given buffer, separated by '\0' characters. */
std::string FormatEigenMatrix(const std::vector<char>& buffer,
                              Eigen::Index rows, Eigen::Index cols);

}  // namespace internal

/** When passing an Eigen::Matrix to fmt, use this wrapper function to instruct
fmt to use Drake's custom formatter for Eigen types.

Within Drake, when formatting an Eigen matrix into a string you must wrap the
Eigen object as `fmt_eigen(M)`. This holds true whether it be for logging, error
messages, debugging, or etc.

For example:
@code
if (!CheckValid(M)) {
  throw std::logic_error(fmt::format("Invalid M = {}", fmt_eigen(M)));
}
@endcode

@warning The return value of this function should only ever be used as a
temporary object, i.e., in a fmt argument list or a logging statement argument
list. Never store it as a local variable, member field, etc.

@note To ensure floating-point data is formatted without losing any digits,
Drake's code is compiled using -DEIGEN_NO_IO, which enforces that nothing within
Drake is allowed to use Eigen's `operator<<`. Downstream code that calls into
Drake is not required to use that option; it is only enforced by Drake's build
system, not by Drake's headers. */
template <typename Derived>
internal::fmt_eigen_ref<Derived> fmt_eigen(
    const Eigen::MatrixBase<Derived>& matrix) {
  return {matrix};
}

}  // namespace drake

#ifndef DRAKE_DOXYGEN_CXX
// Formatter specialization for drake::fmt_eigen.
namespace fmt {
template <typename Derived>
struct formatter<drake::internal::fmt_eigen_ref<Derived>>
    : formatter<typename Derived::Scalar> {
  using Base = formatter<typename Derived::Scalar>;

  template <typename FormatContext>
  auto format(const drake::internal::fmt_eigen_ref<Derived>& ref,
              // NOLINTNEXTLINE(runtime/references) To match fmt API.
              FormatContext& ctx) DRAKE_FMT8_CONST -> decltype(ctx.out()) {
    using Scalar = typename Derived::Scalar;
    const auto& matrix = ref.matrix;
    // Format every matrix element in turn, separated by '\0' characters. Use
    // the Scalar's formatter specialization so its format_spec will be used.
    std::vector<char> formatted_elements;
    formatted_elements.reserve(matrix.size() * 20);
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
        using OutputIt = std::back_insert_iterator<std::vector<char>>;
        using OutputContext = fmt::basic_format_context<OutputIt, char>;
        OutputContext element_ctx{OutputIt(formatted_elements), {}};
        Base::format(matrix(row, col), element_ctx);
        formatted_elements.push_back(0);
      }
    }
    // Layout the matrix data with brackets, commas, whitespace, etc.
    const std::string content = drake::internal::FormatMatrix(
        formatted_elements, matrix.rows(), matrix.cols());
    return formatter<std::string_view>{}.format(content, ctx);
  }
};
}  // namespace fmt
#endif
