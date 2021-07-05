#include <cmath>
#include <memory>

#include <Eigen/Core>

namespace test {
struct CowScalar {
  std::shared_ptr<double> value;

  CowScalar() : value(std::make_shared<double>(0.0)) {}
  CowScalar(double arg) : value(std::make_shared<double>(arg)) {} // NOLINT
  CowScalar(const CowScalar&) = default;
  CowScalar& operator=(const CowScalar&) = default;
  CowScalar(CowScalar&&) = default;
  CowScalar& operator=(CowScalar&&) = default;

  friend CowScalar operator+(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value + *rhs.value;
  }
  friend CowScalar operator+(const CowScalar& other) {
    return +(*other.value);
  }
  friend CowScalar operator-(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value - *rhs.value;
  }
  friend CowScalar operator-(const CowScalar& other) {
    return -(*other.value);
  }
  friend CowScalar operator*(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value * *rhs.value;
  }
  friend CowScalar& operator*=(CowScalar& lhs, const CowScalar& rhs) { // NOLINT
    (*lhs.value) *= *rhs.value;
    return lhs;
  }
  friend CowScalar operator/(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value / *rhs.value;
  }
  friend CowScalar& operator/=(CowScalar& lhs, const CowScalar& rhs) { // NOLINT
    (*lhs.value) /= *rhs.value;
    return lhs;
  }
  friend CowScalar abs(const CowScalar& other) {
    return std::abs(*other.value);
  }
  friend CowScalar sqrt(const CowScalar& other) {
    return std::sqrt(*other.value);
  }
  friend bool operator==(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value == *rhs.value;
  }
  friend bool operator!=(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value != *rhs.value;
  }
  friend bool operator<(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value < *rhs.value;
  }
  friend bool operator<=(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value <= *rhs.value;
  }
  friend bool operator>(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value > *rhs.value;
  }
  friend bool operator>=(const CowScalar& lhs, const CowScalar& rhs) {
    return *lhs.value >= *rhs.value;
  }
  friend bool isfinite(const CowScalar& e) {
    return std::isfinite(*e.value);
  }
};
}  // namespace test

namespace Eigen {
template <>
struct NumTraits<test::CowScalar>
    : GenericNumTraits<test::CowScalar> {
  static inline int digits10() { return 0; }
};

// This guard enables the workaround.
#if 0
namespace internal {
template<> EIGEN_DEVICE_FUNC inline ::test::CowScalar
ptrue(const ::test::CowScalar&) { return 1; }
template<> EIGEN_DEVICE_FUNC inline ::test::CowScalar
pzero(const ::test::CowScalar&) { return 0; }
template<> EIGEN_DEVICE_FUNC inline ::test::CowScalar
pand(const ::test::CowScalar& a, const ::test::CowScalar& b) {
  return static_cast<int>(*a.value) & static_cast<int>(*b.value);
}
template<> EIGEN_DEVICE_FUNC inline ::test::CowScalar
por(const ::test::CowScalar& a, const ::test::CowScalar& b) {
  return static_cast<int>(*a.value) | static_cast<int>(*b.value);
}
template<> EIGEN_DEVICE_FUNC inline ::test::CowScalar
pxor(const ::test::CowScalar& a, const ::test::CowScalar& b) {
  return static_cast<int>(*a.value) ^ static_cast<int>(*b.value);
}
}  // namespace internal
#endif

}  // namespace Eigen

int main() {
  Eigen::Matrix3<test::CowScalar> M;
  M.setIdentity();
  test::CowScalar foo = M.cwiseAbs().template maxCoeff<Eigen::PropagateNaN>();
  if (foo != 1.0) {
    return 1;
  }
  return 0;
}
