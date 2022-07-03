#pragma once

#include <ostream>

#include "drake/common/autodiff/scalar.h"

namespace drake {
namespace autodiff {

inline Scalar operator+(Scalar lhs, const Scalar& rhs) {
  lhs += rhs;
  return lhs;
}

inline Scalar operator+(Scalar lhs, double rhs) {
  lhs += rhs;
  return lhs;
}

inline Scalar operator+(double lhs, Scalar rhs) {
  rhs += lhs;
  return rhs;
}

inline Scalar operator-(Scalar lhs, const Scalar& rhs) {
  lhs -= rhs;
  return lhs;
}

inline Scalar operator-(Scalar lhs, double rhs) {
  lhs -= rhs;
  return lhs;
}

inline Scalar operator-(double lhs, Scalar rhs) {
  rhs *= -1.0;
  rhs += lhs;
  return rhs;
}

inline Scalar operator-(Scalar x) {
  x *= -1.0;
  return x;
}

inline Scalar operator*(Scalar lhs, const Scalar& rhs) {
  lhs *= rhs;
  return lhs;
}

inline Scalar operator*(Scalar lhs, double rhs) {
  lhs *= rhs;
  return lhs;
}

inline Scalar operator*(double lhs, Scalar rhs) {
  rhs *= lhs;
  return rhs;
}

inline Scalar operator/(Scalar lhs, const Scalar& rhs) {
  lhs /= rhs;
  return lhs;
}

inline Scalar operator/(Scalar lhs, double rhs) {
  lhs /= rhs;
  return lhs;
}

inline Scalar operator/(double lhs, const Scalar& rhs) {
  Scalar result{lhs};
  result /= rhs;
  return result;
}

inline bool operator<(const Scalar& lhs, const Scalar& rhs) {
  return lhs.value() < rhs.value();
}

inline bool operator<=(const Scalar& lhs, const Scalar& rhs) {
  return lhs.value() <= rhs.value();
}

inline bool operator>(const Scalar& lhs, const Scalar& rhs) {
  return lhs.value() > rhs.value();
}

inline bool operator>=(const Scalar& lhs, const Scalar& rhs) {
  return lhs.value() >= rhs.value();
}

inline bool operator==(const Scalar& lhs, const Scalar& rhs) {
  return lhs.value() == rhs.value();
}

inline bool operator!=(const Scalar& lhs, const Scalar& rhs) {
  return lhs.value() != rhs.value();
}

inline bool operator<(const Scalar& lhs, double rhs) {
  return lhs.value() < rhs;
}

inline bool operator<=(const Scalar& lhs, double rhs) {
  return lhs.value() <= rhs;
}

inline bool operator>(const Scalar& lhs, double rhs) {
  return lhs.value() > rhs;
}

inline bool operator>=(const Scalar& lhs, double rhs) {
  return lhs.value() >= rhs;
}

inline bool operator==(const Scalar& lhs, double rhs) {
  return lhs.value() == rhs;
}

inline bool operator!=(const Scalar& lhs, double rhs) {
  return lhs.value() != rhs;
}

inline bool operator<(double lhs, const Scalar& rhs) {
  return lhs < rhs.value();
}

inline bool operator<=(double lhs, const Scalar& rhs) {
  return lhs <= rhs.value();
}

inline bool operator>(double lhs, const Scalar& rhs) {
  return lhs > rhs.value();
}

inline bool operator>=(double lhs, const Scalar& rhs) {
  return lhs >= rhs.value();
}

inline bool operator==(double lhs, const Scalar& rhs) {
  return lhs == rhs.value();
}

inline bool operator!=(double lhs, const Scalar& rhs) {
  return lhs != rhs.value();
}

/** ADL overload to mimic std::abs from <cmath>. */
inline Scalar abs(Scalar x) {
  if (x.value() < 0.0) {
    x *= -1.0;
  }
  return x;
}

/** ADL overload to mimic Eigen::numext::abs2. */
inline Scalar abs2(Scalar x) {
  x.ScaleDerivatives(2.0);
  x.value() *= x.value();
  return x;
}

inline Scalar acos(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar asin(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar atan(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar cos(Scalar x) {
  x.ScaleDerivatives(-std::sin(x.value()));
  x.value() = std::cos(x.value());
  return x;
}

inline Scalar cosh(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar exp(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar log(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar sin(Scalar x) {
  x.ScaleDerivatives(std::cos(x.value()));
  x.value() = std::sin(x.value());
  return x;
}

inline Scalar sinh(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar sqrt(Scalar x) {
  const double new_value = std::sqrt(x.value());
  x.ScaleDerivatives(0.5 / new_value);
  x.value() = new_value;
  return x;
}

inline Scalar tan(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar tanh(Scalar x) {
  DRAKE_DEMAND(false);
  return x;
}

inline Scalar atan2(Scalar a, const Scalar& b) {
  a.ScaleDerivatives(b.value());
  a += Scalar{0.0, -a.value() * b.derivatives()};  // XXX
  a.ScaleDerivatives(1.0 / (a.value() * a.value() + b.value() * b.value()));
  a.value() = std::atan2(a.value(), b.value());
  return a;
}

inline Scalar atan2(Scalar a, double b) {
  DRAKE_DEMAND(false);
  (void)(b);
  return a;
}

inline Scalar atan2(double a, Scalar b) {
  DRAKE_DEMAND(false);
  (void)(a);
  return b;
}

inline Scalar max(Scalar a, const Scalar& b) {
  if (b.value() > a.value()) {
    a = b;
  }
  return a;
}

inline Scalar max(Scalar a, double b) {
  if (b > a.value()) {
    a = b;
  }
  return a;
}

inline Scalar max(double a, Scalar b) {
  if (a > b.value()) {
    b = a;
  }
  return b;
}

inline Scalar min(Scalar a, const Scalar& b) {
  if (b.value() < a.value()) {
    a = b;
  }
  return a;
}

inline Scalar min(Scalar a, double b) {
  if (b < a.value()) {
    a = b;
  }
  return a;
}

inline Scalar min(double a, Scalar b) {
  if (a < b.value()) {
    b = a;
  }
  return b;
}

inline Scalar pow(Scalar a, const Scalar& b) {
  DRAKE_DEMAND(false);
  (void)(b);
  return a;
}

inline Scalar pow(Scalar a, double b) {
  DRAKE_DEMAND(false);
  (void)(b);
  return a;
}

inline Scalar pow(double a, Scalar b) {
  DRAKE_DEMAND(false);
  (void)(a);
  return b;
}

/** ADL overload to mimic std::ceil from <cmath>. */
inline double ceil(const Scalar& x) {
  using std::ceil;
  return ceil(x.value());
}

/** ADL overload to mimic std::floor from <cmath>. */
inline double floor(const Scalar& x) {
  using std::floor;
  return floor(x.value());
}

/** ADL overload to mimic std::nexttoward from <cmath>. */
inline double nexttoward(const Scalar& from, long double to) {
  using std::nexttoward;
  return nexttoward(from.value(), to);
}

/** ADL overload to mimic std::round from <cmath>. */
inline double round(const Scalar& x) {
  using std::round;
  return round(x.value());
}

/** ADL overload to mimic std::isfinite from <cmath>. */
inline bool isfinite(const Scalar& x) {
  using std::isfinite;
  return isfinite(x.value());
}

/** ADL overload to mimic std::isinf from <cmath>. */
inline bool isinf(const Scalar& x) {
  using std::isinf;
  return isinf(x.value());
}

/** ADL overload to mimic std::isnan from <cmath>. */
inline bool isnan(const Scalar& x) {
  using std::isnan;
  return isnan(x.value());
}

/** Outputs the value() part of x to the stream. */
inline std::ostream& operator<<(std::ostream& s, const Scalar& x) {
  return s << x.value();
}

}  // namespace autodiff
}  // namespace drake
