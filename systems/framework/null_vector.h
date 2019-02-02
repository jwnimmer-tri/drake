#pragma once

#include "drake/common/never_destroyed.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

/// NullVector is a zero-sized specialization of BasicVector.
template <typename T>
class NullVector final : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NullVector)

  NullVector() : BasicVector<T>(0) {}

  static const NullVector& get() {
    static const never_destroyed<const NullVector> singleton;
    return singleton.access();
  }

 protected:
  BasicVector<T>* DoClone() const override {
    return new NullVector;
  }
};

}  // namespace systems
}  // namespace drake
