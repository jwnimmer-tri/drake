#pragma once

#include <array>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/reset_after_move.h"

namespace drake {
namespace internal {

/* Helper class for LazyMac, below. */
struct LazyMacTerm {
  bool is_zero() const { return coeff == 0.0 || data == nullptr; }

  double coeff{};
  std::shared_ptr<const double[]> data;
};

/* A lazy multiply-accumulate over same-sized vectors, optimized for use with
Drake's autodiff::Scalar.

See https://en.wikipedia.org/wiki/Multiply–accumulate_operation

All vectors must be either the same size or zero.

The amount of laziness is bounded; once we reach the fourth new vector term, we
squash the expression back down to a single vector. */
class LazyMac {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LazyMac);

  LazyMac() = default;

  explicit LazyMac(const Eigen::Ref<const Eigen::VectorXd>& derivatives) {
    if (derivatives.size() > 0) {
      reset(derivatives);
    }
  }

  ~LazyMac() = default;

  void reset() { size_ = 0; }
  void reset(const Eigen::Ref<const Eigen::VectorXd>& derivatives);

  bool empty() const { return size_ == 0; }
  Eigen::Index size() const { return size_; }
  const double* SquashAndGetData();

  void Add(const LazyMac& other) {
    if (!other.empty()) {
      AddImpl(other);
    }
  }

  void Mul(double scale) {
    if (scale == 0.0) {
      size_ = 0;
    } else {
      for (auto& term : terms_) {
        term.coeff *= scale;
      }
    }
  }

  void AddMul(double scale, const LazyMac& other) {
    if (scale != 0.0 && !other.empty()) {
      AddMulImpl(scale, other);
    }
  }

 private:
  void AddImpl(const LazyMac&);
  void AddMulImpl(double, const LazyMac&);

  // Representation invariant:
  // - size_ >= 0
  // - if size_ > 0 then ∀i:
  //   - if terms(i).data is non-null then:
  //     - terms(i).data must contain storage for exactly size_ elements
  // Abstraction function:
  // - When size == 0, the vector's value is zero (aka empty).
  // - When size > 0, the vector's value is Σ term(i).coeff * term(i).data,
  //   where a nullptr term(i).data denotes zero.
  reset_after_move<Eigen::Index> size_{0};
  std::array<LazyMacTerm, 3> terms_;
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
    Eigen::VectorXd derivatives = Eigen::VectorXd::Zero(size);
    derivatives[offset] = 1.0;
    derivatives_.reset(derivatives);
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
    derivatives_.reset();
    return *this;
  }

  ~Scalar() = default;

  double& value() { return value_; }
  double value() const { return value_; }

  Eigen::Map<const Eigen::VectorXd> derivatives() const {
    // [insert scary comment here]
    auto& mutable_mac = const_cast<internal::LazyMac&>(derivatives_);
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
  internal::LazyMac derivatives_{};
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
