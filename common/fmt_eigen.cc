#include "drake/common/fmt_eigen.h"

#include <algorithm>
#include <cstring>

#include "drake/common/eigen_types.h"

namespace drake {
namespace internal {
namespace {

/* Returns a row string like `[0, 1, 2, ...]` for the given element strings.
Elements will be left-padded with spaces to reach `minimum_element_width`. */
std::string FormatRow(const VectorX<const char*>& row_elements,
                      size_t minimum_element_width = 0) {
  // TODO(jwnimmer-tri) Once we're at Eigen >= 3.4 on all platforms, we can
  // use its built-in iterators here instead of copying it.
  const std::vector<const char*> iterable_vector(
      row_elements.data(), row_elements.data() + row_elements.size());
  return fmt::format("[{0:>{1}}]", fmt::join(iterable_vector, ", "),
                     minimum_element_width);
}

}  // namespace

std::string FormatMatrix(const std::vector<char>& buffer, Eigen::Index rows,
                         Eigen::Index cols) {
  // Short-circuit for an empty matrix.
  if (rows == 0 || cols == 0) {
    DRAKE_DEMAND(buffer.empty());
    return "[]";
  }
  DRAKE_DEMAND(!buffer.empty());

  // Locate all of the element strings within the buffer.
  MatrixX<const char*> elements(rows, cols);
  size_t max_width = 0;
  const char* iter = buffer.data();
  const char* const end = buffer.data() + buffer.size();
  for (Eigen::Index row = 0; row < rows; ++row) {
    for (Eigen::Index col = 0; col < cols; ++col) {
      DRAKE_DEMAND(iter < end);
      elements(row, col) = iter;
      const size_t width = std::strlen(iter);
      max_width = std::max(width, max_width);
      iter += width + 1;
    }
  }
  DRAKE_DEMAND(iter == end);

  // Display row-vectors horizontally.
  if (rows == 1) {
    return FormatRow(elements.row(0));
  }

  // Display column-vectors horizontally (i.e., transposed).
  if (cols == 1) {
    std::string result = FormatRow(elements.col(0).transpose());
    result.append("ᵀ");
    return result;
  }

  // Display 2d matrices with fixed-width columns, using the maximum width of
  // any one element as all columns' width.
  std::vector<std::string> formatted_rows;
  formatted_rows.reserve(rows);
  for (Eigen::Index row = 0; row < rows; ++row) {
    formatted_rows.push_back(FormatRow(elements.row(row), max_width));
  }
  return fmt::format("[{}]", fmt::join(formatted_rows, ",\n "));
}

}  // namespace internal
}  // namespace drake
