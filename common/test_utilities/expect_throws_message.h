#pragma once

#include <ostream>
#include <regex>
#include <string>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/nice_type_name.h"

// TODO(sherm1) Add unit tests for these macros. See issue #8403.

#ifdef DRAKE_DOXYGEN_CXX
/** Unit test helper macro for "expecting" an exception to be thrown but also
testing the error message against a provided regular expression. This is
like GTest's `EXPECT_THROW` but is fussier about the particular error message.
Usage example: @code
  DRAKE_EXPECT_THROWS_MESSAGE(
      StatementUnderTest(), // You expect this statement to throw ...
      std::logic_error,     // ... this exception with ...
      ".*some important.*phrases.*that must appear.*");  // ... this message.
@endcode
The regular expression must match the entire error message. If there is
boilerplate you don't care to match at the beginning and end, surround with
`.*` to ignore.

Following GTest's conventions, failure to perform as expected here is a
non-fatal test error. An `ASSERT` variant is provided to make it fatal. Also,
we provide `IF_ARMED` variants for testing error messages that are thrown only
in Debug builds (or any builds where `DRAKE_ENABLE_ASSERTS` has been defined).
@see DRAKE_ASSERT_THROWS_MESSAGE
@see DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED, DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED */
#define DRAKE_EXPECT_THROWS_MESSAGE(expression, exception, regexp)

/** Fatal error version of `DRAKE_EXPECT_THROWS_MESSAGE`.
@see DRAKE_EXPECT_THROWS_MESSAGE */
#define DRAKE_ASSERT_THROWS_MESSAGE(expression, exception, regexp)

/** Same as `DRAKE_EXPECT_THROWS_MESSAGE` in Debug builds, but doesn't require
a throw in Release builds. However, if the Release build does throw it must
throw the right message. More precisely, the thrown message is required
whenever `DRAKE_ENABLE_ASSERTS` is defined, which Debug builds do be default.
@see DRAKE_EXPECT_THROWS_MESSAGE */
#define DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp)

/** Same as `DRAKE_ASSERT_THROWS_MESSAGE` in Debug builds, but doesn't require
a throw in Release builds. However, if the Release build does throw it must
throw the right message. More precisely, the thrown message is required
whenever `DRAKE_ENABLE_ASSERTS` is defined, which Debug builds do be default.
@see DRAKE_ASSERT_THROWS_MESSAGE */
#define DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp)
#endif

#ifndef DRAKE_DOXYGEN_CXX
#define DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                           must_throw, fatal_failure) \
try { \
  expression; \
  if (must_throw) { \
    if (fatal_failure) { \
      GTEST_FATAL_FAILURE_("\t" #expression " failed to throw " #exception); \
    } else { \
      GTEST_NONFATAL_FAILURE_("\t" #expression " failed to throw " #exception);\
    } \
  } \
} catch (const exception& err) { \
  auto matcher = [](const char* s, const std::string& re) { \
    return std::regex_match(s, std::regex(re)); }; \
  if (fatal_failure) { \
    ASSERT_PRED2(matcher, err.what(), regexp); \
  } else { \
    EXPECT_PRED2(matcher, err.what(), regexp); \
  } \
}

#define DRAKE_EXPECT_THROWS_MESSAGE(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     true /*must_throw*/, false /*non-fatal*/)

#define DRAKE_ASSERT_THROWS_MESSAGE(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     true /*must_throw*/, true /*fatal*/)

#ifdef DRAKE_ASSERT_IS_DISARMED
// Throwing the expected message is optional in this case.

#define DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     false /*optional*/, false /*non-fatal*/)

#define DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE_HELPER(expression, exception, regexp, \
                                     false /*optional*/, true /*fatal*/)

#else
// Throwing the expected message is required in this case.

#define DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp) \
  DRAKE_EXPECT_THROWS_MESSAGE(expression, exception, regexp)

#define DRAKE_ASSERT_THROWS_MESSAGE_IF_ARMED(expression, exception, regexp) \
  DRAKE_ASSERT_THROWS_MESSAGE(expression, exception, regexp)

#endif
#endif

namespace drake {
namespace test {

/** XXX */
struct ReifiedException {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ReifiedException)

  const std::type_info* rtti{};
  std::string rtti_name;

  std::string what;

  using first_type = const std::type_info*;
  using second_type = std::string;
  const std::type_info* first{};
  std::string second;

  ReifiedException() : ReifiedException(nullptr, {}) {}
  // NOLINTNEXTLINE(runtime/explicit) Our macro relies on this.
  ReifiedException(const std::exception& e)
      : ReifiedException(&typeid(e), e.what()) {}
  ReifiedException(const std::type_info* rtti_in, std::string what_in)
      : rtti(rtti_in ? rtti_in : &typeid(void)),
        rtti_name(::drake::NiceTypeName::Get(*rtti)),
        what(std::move(what_in)),
        first(rtti), second(what) {}

  friend void PrintTo(const ReifiedException& x, std::ostream* os) {
    *os << x.rtti_name << "(\"" << x.what << "\")";
  }
};

}  // namespace test

template <typename T, typename WhatMatcher>
auto IsException(WhatMatcher what_matcher) {
  return ::testing::Pair(&typeid(T), what_matcher);
}

}  // namespace drake

// We're not supposed to add things to std namespace, but there doesn't seem to
// be a better way to get this to print well in googletest.
namespace std {
inline void PrintTo(const std::type_info* x, std::ostream* os) {
  if (x) {
    *os << "&typeid(" << ::drake::NiceTypeName::Get(*x) << ")";
  } else {
    *os << "nullptr";
  }
}
}  // namespace std

// N.B. We can't use EXPECT_EQ on the result of this macro, because C++ doesn't
// support lambdas in un-evaluated contexts and EXPECT_EQ uses its arguments in
// such a context.
#define EXCEPTION_THROWN_BY(expression)         \
  [&]() -> ::drake::test::ReifiedException {    \
    try {                                       \
      (expression);                             \
    } catch (const std::exception& e) {         \
      return e;                                 \
    }                                           \
    return {};                                  \
  }()

#define EXPECT_THROW2(expression, type, ...) \
  EXPECT_THAT(EXCEPTION_THROWN_BY(expression), IsException<type>(__VA_ARGS__))

#define EXPECT_THAT_THROWN_MESSAGE(expression, ...) \
  EXPECT_THAT(EXCEPTION_THROWN_BY(expression).what, __VA_ARGS__)

#ifdef DRAKE_ASSERT_IS_ARMED
template <typename T>
auto IfAssertArmed(T t) {
  return t;
}
#else
template <typename T>
auto IfAssertArmed(T) {
  return ::testing::_;
}
#endif
