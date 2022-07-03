#pragma once

#include <array>
#include <atomic>
#include <cstdlib>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_after_move.h"

namespace drake {
namespace internal {

/* Helper class for Partials, below. */
class CowVec {
 public:
  CowVec() = default;

  static CowVec Allocate(Eigen::Index size);
  static CowVec Copy(const Eigen::Ref<const Eigen::VectorXd>& derivatives);

  CowVec(CowVec&& other) noexcept {
    std::swap(data_, other.data_);
  }

  CowVec& operator=(CowVec&& other) noexcept {
    std::swap(data_, other.data_);
    return *this;
  }

  CowVec(const CowVec& other) noexcept {
    if (other.data_ != nullptr) {
      ++other.use_count();
      data_ = other.data_;
    }
  }

  CowVec& operator=(const CowVec& other) noexcept {
    if (this != &other) {
      if (other.data_ != nullptr) {
        ++other.use_count();
        data_ = other.data_;
      }
    }
    return *this;
  }

  ~CowVec() {
    if (data_ != nullptr) {
      UseCount& counter = use_count();
      if (--counter == 0) {
	counter.~UseCount();
	std::free(&counter);
      }
    }
  }

  const double* data() const { return data_; }

  // XXX rename me (or inline)?
  double* mutable_data(Eigen::Index size);

  // XXX private
  using UseCount = std::atomic_int_fast64_t;
  UseCount& use_count() const {
    DRAKE_ASSERT(data_ != nullptr);
    return *reinterpret_cast<UseCount*>(
        reinterpret_cast<char*>(data_) - sizeof(UseCount));
  }

 private:
  double* data_{nullptr};
};

/* A lazy multiply-accumulate over same-sized vectors, optimized for use with
Drake's autodiff::Scalar.

All vectors must be either the same size or zero. */
class Partials {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Partials);

  Partials() = default;

  explicit Partials(const Eigen::Ref<const Eigen::VectorXd>& derivatives);

  ~Partials() = default;

  void clear() {
    size_ = 0;
    coeff_ = 0.0;
    storage_ = {};
  }

  bool empty() const { return size_ == 0; }
  Eigen::Index size() const { return size_; }
  double* SquashAndGetData();

  void Add(const Partials& other) {
    if (!other.empty()) {
      AddImpl(other);
    }
  }

  void Mul(double scale) {
    if (scale == 0.0) {
      size_ = 0;
      coeff_ = 0.0;
    } else {
      coeff_ *= scale;
    }
  }

  void AddMul(double scale, const Partials& other) {
    if (scale != 0.0 && !other.empty()) {
      AddMulImpl(scale, other);
    }
  }

 private:
  void CheckInvariants() const;
  void AddImpl(const Partials&);
  void AddMulImpl(double, const Partials&);

  // Representation invariant:
  // - size_ >= 0
  // - if size_ > 0 then ∀i:
  //   - coeff != 0
  //   - if term.data is non-null then:
  //     - term.data must contain storage for exactly size_ elements
  // Abstraction function:
  // - When size == 0, the vector's value is zero (aka empty).
  // - When size > 0, the vector's value is coeff * term.data,
  //   where a nullptr term.data denotes a zero vector.
  reset_after_move<Eigen::Index> size_{0};
  reset_after_move<double> coeff_{0.0};
  CowVec storage_;
};

}  // namespace internal

namespace autodiff {

class Scalar {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Scalar);

  // TODO(jwnimmer-tri) Murder this.
  using DerType = Eigen::VectorXd;

  /** Constructs zero. */
  Scalar() = default;

  /** Constructs a value with empty derivatives. */
  // NOLINTNEXTLINE(runtime/explicit): This conversion is desirable.
  Scalar(double value) : value_{value} {}

  Scalar(double value, Eigen::Index size, Eigen::Index offset)
      : value_{value} {
    // XXX we could optimize this, but it's rarely used...?
    Eigen::VectorXd unit = Eigen::VectorXd::Zero(size);
    unit[offset] = 1.0;
    derivatives_ = internal::Partials(unit);
  }

  /** Constructs a value with given derivatives. */
  Scalar(
      double value,
      const Eigen::Ref<const Eigen::VectorXd>& derivatives)
      : value_{value},
	derivatives_{derivatives} {}

  /** Assigns a value and clears the derivatives. */
  Scalar& operator=(double value) {
    value_ = value;
    derivatives_.clear();
    return *this;
  }

  ~Scalar() = default;

  double& value() { return value_; }
  double value() const { return value_; }

  Eigen::Map<const Eigen::VectorXd> derivatives() const {
    // [insert scary comment here]
    auto& mutable_mac = const_cast<internal::Partials&>(derivatives_);
    const double* data = mutable_mac.SquashAndGetData();
    const Eigen::Index size = derivatives_.size();
    return Eigen::VectorXd::Map(data, size);
  }

  Scalar& operator+=(const Scalar& rhs) {
    derivatives_.Add(rhs.derivatives_);
    value_ += rhs.value_;
    return *this;
  }

  Scalar& operator+=(double rhs) {
    value_ += rhs;
    return *this;
  }

  Scalar& operator-=(const Scalar& rhs) {
    derivatives_.AddMul(-1.0, rhs.derivatives_);
    value_ -= rhs.value_;
    return *this;
  }

  Scalar& operator-=(double rhs) {
    value_ -= rhs;
    return *this;
  }

  Scalar& operator*=(const Scalar& rhs) {
    derivatives_.Mul(rhs.value_);
    derivatives_.AddMul(value_, rhs.derivatives_);
    value_ *= rhs.value_;
    return *this;
  }

  Scalar& operator*=(double rhs) {
    derivatives_.Mul(rhs);
    value_ *= rhs;
    return *this;
  }

  Scalar& operator/=(const Scalar& rhs) {
    derivatives_.Mul(rhs.value_);
    derivatives_.AddMul(-value_, rhs.derivatives_);
    derivatives_.Mul(1.0 / (rhs.value_ * rhs.value_));
    value_ /= rhs.value_;
    return *this;
  }

  Scalar& operator/=(double rhs) {
    return (*this) *= (1.0 / rhs);
  }

  void ScaleDerivatives(double scale) {
    derivatives_.Mul(scale);
  }

 private:
  double value_{0.0};
  internal::Partials derivatives_{};
};

}  // namespace autodiff
}  // namespace drake

namespace Eigen {

template <>
struct NumTraits<drake::autodiff::Scalar> : public NumTraits<double> {
  using Real = drake::autodiff::Scalar;
  using NonInteger = Real;
  using Nested = Real;
  using Literal = double;
  enum {
    RequireInitialization = 1
  };
};

template <typename BinOp>
struct ScalarBinaryOpTraits<drake::autodiff::Scalar, double, BinOp> {
  using ReturnType = drake::autodiff::Scalar;
};

template <typename BinOp>
struct ScalarBinaryOpTraits<double, drake::autodiff::Scalar, BinOp> {
  using ReturnType = drake::autodiff::Scalar;
};

template <typename BinOp>
struct ScalarBinaryOpTraits<drake::autodiff::Scalar, drake::autodiff::Scalar, BinOp> {
  using ReturnType = drake::autodiff::Scalar;
};

}  // namespace Eigen
