#include <gtest/gtest.h>

#include "drake/common/ad/auto_diff.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

using Eigen::Vector3d;
using test::LimitMalloc;

GTEST_TEST(MatrixHeapTest, MatrixProduct) {
  const AutoDiff a00{0.1, Vector3d::LinSpaced(1.0, 2.0)};
  const AutoDiff a01{0.2, Vector3d::LinSpaced(2.0, 3.0)};
  const AutoDiff a02{0.3, Vector3d::LinSpaced(3.0, 4.0)};
  const AutoDiff a10{0.4, Vector3d::LinSpaced(4.0, 5.0)};
  const AutoDiff a11{0.5, Vector3d::LinSpaced(5.0, 6.0)};
  const AutoDiff a12{0.6, Vector3d::LinSpaced(6.0, 7.0)};

  const AutoDiff b00{0.7, Vector3d::LinSpaced(7.0, 8.0)};
  const AutoDiff b10{0.8, Vector3d::LinSpaced(8.0, 9.0)};
  const AutoDiff b20{0.9, Vector3d::LinSpaced(9.0, 10.0)};

  // clang-format off
  MatrixX<AutoDiff> A(2, 3);
  A << a00, a01, a02,
       a10, a11, a12;
  MatrixX<AutoDiff> B(3, 1);
  B << b00,
       b10,
       b20;
  // clang-format on

  // Pre-allocate storage for X = A * B.
  MatrixX<AutoDiff> X;
  X.resize(2, 1);

  // The expected allocations are:
  // - a00*b00 map_base_evaluator copy of each operand
  // - a01*b10 "
  // - a02*b20 "
  // - a10*b00 "
  // - a11*b10 "
  // - a12*b20 "
  // - x00 assign_evaluator copy of the rhs
  // - x10 assign_evaluator copy of the rhs
  LimitMalloc guard({14});
  X.noalias() = A * B;
}

}  // namespace
}  // namespace internal
}  // namespace ad
}  // namespace drake
