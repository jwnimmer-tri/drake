#include "drake/common/fmt.h"

#include <ostream>

#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

// This namespace contains example code (i.e., what a user would write) for
// formatting a class (or struct) using the "format as" helper.
namespace sample {

// A simple, non-templated struct.
struct Int {
  int i{};
};

// A template with 1 type argument.
template <typename T>
struct Wrap {
  T t{};
};

// A template with 2 type arguments.
template <typename T, typename U>
struct Pair {
  T t{};
  U u{};
};

}  // namespace sample

// Tell fmt how to format the sample types.
DRAKE_FORMATTER_AS(, sample, Int, x, x.i)
DRAKE_FORMATTER_AS(typename T, sample, Wrap<T>, x, x.t)
DRAKE_FORMATTER_AS(typename... Ts, sample, Pair<Ts...>, x, std::pair(x.t, x.u))

// Sample code (i.e., what a user would write) for repr-enabled formatting of a
// class (or struct).
namespace sample {
struct Quux {
  int value{};
};
}  // namespace sample
namespace fmt {
template <>
struct formatter<sample::Quux> : drake::repr_formatter {
  template <typename FormatContext>
  auto format(const sample::Quux& quux,
              // NOLINTNEXTLINE(runtime/references) To match fmt API.
              FormatContext& ctx) DRAKE_FMT8_CONST {
    if (repr()) {
      return Base::format(fmt::format("Quux({})", quux.value), ctx);
    } else {
      return Base::format(fmt::format("{}", quux.value), ctx);
    }
  }
};
}  // namespace fmt

// Sample code (i.e., what a user would write) for repr-enabled Enum formatting.
namespace sample {
enum class MyEnum { kFoo, kBar };
std::string to_string(MyEnum value) {
  switch (value) {
    case MyEnum::kFoo: return "kFoo";
    case MyEnum::kBar: return "kBar";
  }
  DRAKE_UNREACHABLE();
}
}  // namespace sample
namespace fmt {
template <>
struct formatter<sample::MyEnum> : drake::repr_formatter {
  template <typename FormatContext>
  auto format(const sample::MyEnum& my_enum,
              // NOLINTNEXTLINE(runtime/references) To match fmt API.
              FormatContext& ctx) DRAKE_FMT8_CONST {
    const auto& enum_str = sample::to_string(my_enum);
    if (repr()) {
      return Base::format("MyEnum::" + enum_str, ctx);
    } else {
      return Base::format(enum_str, ctx);
    }
  }
};
}  // namespace fmt

namespace drake {
namespace {

// Spot check the for "format as" formatter.
GTEST_TEST(FmtTest, FormatAsFormatter) {
  const sample::Int plain{1};
  EXPECT_EQ(fmt::format("{}", plain), "1");
  EXPECT_EQ(fmt::format("{:3}", plain), "  1");
  EXPECT_EQ(fmt::format("{:<3}", plain), "1  ");

  const sample::Wrap<double> real{1.1234567e6};
  EXPECT_EQ(fmt::format("{}", real), "1123456.7");
  EXPECT_EQ(fmt::format("{:.3G}", real), "1.12E+06");

  // N.B. The fmt:formatter for std::pair comes from <fmt/ranges.h>.
  const sample::Pair<int, int> pear{1, 2};
  EXPECT_EQ(fmt::format("{}", pear), "(1, 2)");
}

// Acceptance test for drake::repr_formatter on a struct.
GTEST_TEST(FmtTest, ReprFormatter) {
  // Without repr-level details.
  const sample::Quux value{1};
  EXPECT_EQ(fmt::format("{}", value), "1");
  EXPECT_EQ(fmt::format("{:3}", value), "1  ");
  EXPECT_EQ(fmt::format("{:>3}", value), "  1");

  // With repr-level details.
  EXPECT_EQ(fmt::format("{:!r}", value), "Quux(1)");
  EXPECT_EQ(fmt::format("{:!r9}", value), "Quux(1)  ");
  EXPECT_EQ(fmt::format("{:!r>9}", value), "  Quux(1)");
}

// Acceptance test for drake::to_string_formatter on an enum.
GTEST_TEST(FmtTest, ToStringFormatter) {
  const sample::MyEnum value = sample::MyEnum::kFoo;
  EXPECT_EQ(fmt::format("{}", value), "kFoo");
  EXPECT_EQ(fmt::format("{:6}", value), "kFoo  ");
  EXPECT_EQ(fmt::format("{:>6}", value), "  kFoo");

  EXPECT_EQ(fmt::format("{}", sample::MyEnum::kBar), "kBar");

  EXPECT_EQ(fmt::format("{:!r}", value), "MyEnum::kFoo");
  EXPECT_EQ(fmt::format("{:!r>14}", value), "  MyEnum::kFoo");
}

// The googletest infrastructure uses fmt's formatters.
GTEST_TEST(FmtTest, TestPrinter) {
  const sample::Int plain{1};
  EXPECT_EQ(testing::PrintToString(plain), "1");

  const sample::Wrap<double> real{1.1};
  EXPECT_EQ(testing::PrintToString(real), "1.1");

  const sample::Pair<int, int> pear{1, 2};
  EXPECT_EQ(testing::PrintToString(pear), "(1, 2)");

  const sample::Quux quux{2};
  EXPECT_EQ(testing::PrintToString(quux), "Quux(2)");

  const sample::MyEnum foo = sample::MyEnum::kFoo;
  EXPECT_EQ(testing::PrintToString(foo), "MyEnum::kFoo");
}

}  // namespace
}  // namespace drake
