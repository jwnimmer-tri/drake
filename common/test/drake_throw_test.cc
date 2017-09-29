#include "drake/common/drake_throw.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace {

GTEST_TEST(DrakeThrowTest, BasicTest) {
  EXPECT_NO_THROW(DRAKE_THROW_UNLESS(true));
  EXPECT_THROW(DRAKE_THROW_UNLESS(false), std::runtime_error);
}

GTEST_TEST(DrakeThrowTest, EqTest) {
  EXPECT_NO_THROW(DRAKE_THROW_UNLESS_EQ(1.0, 1.0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      { DRAKE_THROW_UNLESS_EQ(1.0, 2.0); },
      std::runtime_error,
      "Failure.*drake_throw_test.*: condition '1.0 == 2.0' failed. "
      "The operands were '1.000000' and '2.000000'.");
}

}  // namespace
