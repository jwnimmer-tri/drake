#pragma once

#include <iterator>
#include <string_view>
#include <utility>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace solvers {

#ifndef DRAKE_DOXYGEN_CXX
class SolverOptions;  // Defined in solver_options.h.
#endif

/** (Advanced) A LegacyInputIterator for SolverOptions values.
@see SolverOptions::GetRange()
@tparam T can be int, double, std::string_view, std::string, or `const char*` */
template <typename T>
class SolverOptionsIterator {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverOptionsIterator)

  SolverOptionsIterator() = default;

  // Per https://en.cppreference.com/w/cpp/iterator/iterator_traits.
  using difference_type = std::ptrdiff_t;
  using value_type = std::pair<std::string_view, T>;
  using pointer = void;
  using reference = value_type;
  using iterator_category = std::input_iterator_tag;

  // Per https://en.cppreference.com/w/cpp/named_req/Iterator and
  // https://en.cppreference.com/w/cpp/named_req/InputIterator.
  value_type operator*() const;
  const SolverOptionsIterator& operator++();
  bool operator==(const SolverOptionsIterator& other) {
    return (parent_ == other.parent_) && (index_ == other.index_);
  }
  bool operator!=(const SolverOptionsIterator& other) {
    return !(*this == other);
  }

 private:
  friend class SolverOptions;

  SolverOptionsIterator(const SolverOptions* parent, int index);

  const SolverOptions* parent_{nullptr};
  int index_{0};
};

/** (Advanced) A read-only range for SolverOptions values.
@see SolverOptions::GetRange()
@tparam T can be int, double, std::string_view, std::string, or `const char*` */
template <typename T>
class SolverOptionsRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverOptionsRange)

  /** Constructs an empty range. */
  SolverOptionsRange() = default;

  /** Constructs a range starting from `begin`. */
  explicit SolverOptionsRange(const SolverOptionsIterator<T>& begin)
      : begin_(begin) {}

  SolverOptionsIterator<T> begin() const { return begin_; }
  SolverOptionsIterator<T> end() const { return {}; }

 private:
  SolverOptionsIterator<T> begin_;
};

}  // namespace solvers
}  // namespace drake
