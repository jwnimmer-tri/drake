#pragma once

#include <initializer_list>
#include <ostream>
#include <utility>


#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/// VectorBase is an abstract base class that real-valued signals between
/// Systems and real-valued System state vectors must implement.  Classes that
/// inherit from VectorBase will typically provide names for the elements of
/// the vector, and may also provide other computations for the convenience of
/// Systems handling the signal. The vector is always a column vector. It may
/// or may not be contiguous in memory. Contiguous subclasses should typically
/// inherit from BasicVector, not from VectorBase directly.
///
/// @tparam_default_scalar
template <typename T>
class VectorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorBase)

  virtual ~VectorBase();

  /// Returns the number of elements in the vector.
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual int size() const = 0;

  /// Returns the element at the given index in the vector.
  /// @pre 0 <= `index` < size()
  T& operator[](int index) {
    DRAKE_ASSERT(index >= 0);
    return DoGetAtIndex(index);
  }

  /// Returns the element at the given index in the vector.
  /// @pre 0 <= `index` < size()
  const T& operator[](int index) const {
    DRAKE_ASSERT(index >= 0);
    return DoGetAtIndex(index);
  }

  /// Returns the element at the given index in the vector.
  /// @throws std::runtime_error if the index is >= size() or negative.
  /// Consider operator[]() instead if bounds-checking is unwanted.
  const T& GetAtIndex(int index) const {
    if (index < 0) { this->ThrowOutOfRange(index); }
    return DoGetAtIndex(index);
  }

  /// Returns the element at the given index in the vector.
  /// @throws std::runtime_error if the index is >= size() or negative.
  /// Consider operator[]() instead if bounds-checking is unwanted.
  T& GetAtIndex(int index) {
    if (index < 0) { this->ThrowOutOfRange(index); }
    return DoGetAtIndex(index);
  }

  /// Replaces the state at the given index with the value.
  /// @throws std::runtime_error if the index is >= size().
  /// Consider operator[]() instead if bounds-checking is unwanted.
  void SetAtIndex(int index, const T& value) {
    GetAtIndex(index) = value;
  }

  /// Replaces the entire vector with the contents of @p value.
  /// @throws std::runtime_error if @p value is not a column vector with size()
  /// rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFrom(const VectorBase<T>& value);

  /// Replaces the entire vector with the contents of @p value. Throws
  /// std::runtime_error if @p value is not a column vector with size() rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFromVector(const Eigen::Ref<const VectorX<T>>& value);

  /// Sets all elements of this vector to zero.
  virtual void SetZero();

  /// Copies this entire %VectorBase into a contiguous Eigen Vector.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates only the O(N) memory that it returns.
  virtual VectorX<T> CopyToVector() const;

  /// Copies this entire %VectorBase into a pre-sized Eigen Vector.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value.
  /// @throws std::exception if `vec` is the wrong size.
  virtual void CopyToPreSizedVector(EigenPtr<VectorX<T>> vec) const;

  /// Adds a scaled version of this vector to Eigen vector @p vec, which
  /// must be the same size.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual void ScaleAndAddToVector(const T& scale,
                                   EigenPtr<VectorX<T>> vec) const;

  /// Add in scaled vector @p rhs to this vector. Both vectors must
  /// be the same size.
  VectorBase& PlusEqScaled(const T& scale, const VectorBase<T>& rhs) {
    return PlusEqScaled({{scale, rhs}});
  }

  /// Add in multiple scaled vectors to this vector. All vectors
  /// must be the same size.
  VectorBase& PlusEqScaled(const std::initializer_list<
                           std::pair<T, const VectorBase<T>&>>& rhs_scale) {
    const int n = size();
    for (const auto& operand : rhs_scale) {
      const int rhs_n = operand.second.size();
      if (rhs_n != n) { ThrowMismatchedSize(rhs_n); }
    }
    DoPlusEqScaled(rhs_scale);
    return *this;
  }

  /// Add in vector @p rhs to this vector.
  VectorBase& operator+=(const VectorBase<T>& rhs) {
    return PlusEqScaled(T(1), rhs);
  }

  /// Subtract in vector @p rhs to this vector.
  VectorBase& operator-=(const VectorBase<T>& rhs) {
    return PlusEqScaled(T(-1), rhs);
  }

  /// Get the bounds for the elements.
  /// If lower and upper are both empty size vectors, then there are no bounds.
  /// Otherwise, the bounds are (*lower)(i) <= GetAtIndex(i) <= (*upper)(i)
  /// The default output is no bounds.
  virtual void GetElementBounds(Eigen::VectorXd* lower,
                                Eigen::VectorXd* upper) const;

 protected:
  VectorBase() = default;

  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.  The index has already been checked for negative, but not size.
  virtual const T& DoGetAtIndex(int index) const = 0;

  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.  The index has already been checked for negative, but not size.
  virtual T& DoGetAtIndex(int index) = 0;

  /// Adds in multiple scaled vectors to this vector. All vectors
  /// are guaranteed to be the same size.
  ///
  /// You should override this method if possible with a more efficient
  /// approach that leverages structure; the default implementation performs
  /// element-by-element computations that are likely inefficient, but even
  /// this implementation minimizes memory accesses for efficiency. If the
  /// vector is contiguous, for example, implementations that leverage SIMD
  /// operations should be far more efficient. Overriding implementations should
  /// ensure that this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual void DoPlusEqScaled(const std::initializer_list<
                              std::pair<T, const VectorBase<T>&>>& rhs_scale);

  [[noreturn]] void ThrowOutOfRange(int index) const;
  [[noreturn]] void ThrowMismatchedSize(int other_size) const;
};

/// Allows a VectorBase<T> to be streamed into a string as though it were a
/// RowVectorX<T>. This is useful for debugging purposes.
template <typename T>
std::ostream& operator<<(std::ostream& os, const VectorBase<T>& vec) {
  os << vec.CopyToVector().transpose();
  return os;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorBase)
