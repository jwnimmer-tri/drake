#include "drake/common/test_utilities/expect_throws_message.h"

#include <stdexcept>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace {

using ::testing::Eq;
using ::testing::ContainsRegex;
using ::testing::MatchesRegex;

// This whole file is more of a "does it compile" acceptance test and demo,
// than a functional unit test.

void Boom() {
  throw std::runtime_error("a message");
}

void BoomIfArmed() {
#ifdef DRAKE_ASSERT_IS_ARMED
  Boom();
#endif
}

// Check the message that was thrown, as a literal.
GTEST_TEST(ExpectThrowsMessageTest, Message) {
  EXPECT_THAT_THROWN_MESSAGE(Boom(), Eq("a message"));
  EXPECT_THAT_THROWN_MESSAGE(Boom(), "a message");  // Eq is the default.
}

// Check the message that was thrown, using a regex.
GTEST_TEST(ExpectThrowsMessageTest, MessageRegex) {
  EXPECT_THAT_THROWN_MESSAGE(Boom(), MatchesRegex("a.*ge"));
  EXPECT_THAT_THROWN_MESSAGE(Boom(), ContainsRegex("mess"));
}

// Check the type that was thrown.
GTEST_TEST(ExpectThrowsMessageTest, Type) {
  // Already built-in to googletest.
  EXPECT_THROW(Boom(), std::exception);
  EXPECT_THROW(Boom(), std::runtime_error);

  // New in Drake -- check for the most specific derived type by string.
  EXPECT_THAT(EXCEPTION_THROWN_BY(Boom()).rtti_name, "std::runtime_error");
}

// Check both the type and the message.
GTEST_TEST(ExpectThrowsMessageTest, MessageAndType) {
  EXPECT_THROW2(Boom(), std::runtime_error, "a message");
  EXPECT_THROW2(Boom(), std::runtime_error, MatchesRegex("a.*ge"));
}

// Check for a message conditionally thrown.
GTEST_TEST(ExpectThrowsMessageTest, DebugConditional) {
  EXPECT_THAT_THROWN_MESSAGE(BoomIfArmed(), IfAssertArmed(Eq("a message")));
}

}  // namespace
}  // namespace drake
