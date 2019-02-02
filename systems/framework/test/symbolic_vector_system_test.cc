#include "drake/systems/framework/symbolic_vector_system.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/gen/vec1.h"
#include "drake/systems/framework/test_utilities/gen/vec2.h"
#include "drake/systems/framework/test_utilities/gen/vec3.h"

using V1d = drake::Vector1d;
using V2d = Eigen::Vector2d;

namespace drake {
namespace systems {
namespace {

using ParameterSize = TypeSafeIndex<class ParameterSizeTag>;
using InputSize = TypeSafeIndex<class InputSizeTag>;
using ContinuousStateSize = TypeSafeIndex<class ContinuousStateSizeTag>;
using DiscreteStateSize = TypeSafeIndex<class DiscreteStateSizeTag>;
using OutputSize = TypeSafeIndex<class OutputSizeTag>;

void CheckShape(const SymbolicVectorSystem& dut, ParameterSize size) {
  const auto& context = dut.CreateDefaultContext();
  if (size == 0) {
    ASSERT_EQ(context->num_numeric_parameters(), 0);
  } else {
    ASSERT_EQ(context->num_numeric_parameters(), 1);
    ASSERT_EQ(context->get_numeric_parameter(0).size(), size);
  }
}

void CheckShape(const SymbolicVectorSystem& dut, InputSize size) {
  if (size == 0) {
    ASSERT_EQ(dut.get_num_input_ports(), 0);
  } else {
    ASSERT_EQ(dut.get_num_input_ports(), 1);
    const auto& descriptor = dut.get_input_port();
    EXPECT_EQ(descriptor.get_data_type(), kVectorValued);
    EXPECT_EQ(descriptor.size(), size);
  }
}

void CheckShape(const SymbolicVectorSystem& dut, ContinuousStateSize size) {
  const auto& context = dut.CreateDefaultContext();
  ASSERT_EQ(context->get_continuous_state().size(), size);
}

void CheckShape(const SymbolicVectorSystem& dut, DiscreteStateSize size) {
  const auto& context = dut.CreateDefaultContext();
  const auto& discrete_state = context->get_discrete_state();
  if (size == 0) {
    ASSERT_EQ(discrete_state.num_groups(), 0);
  } else {
    ASSERT_EQ(discrete_state.num_groups(), 1);
    ASSERT_EQ(discrete_state.get_vector(0).size(), size);
  }
}

void CheckShape(const SymbolicVectorSystem& dut, OutputSize size) {
  if (size == 0) {
    ASSERT_EQ(dut.get_num_output_ports(), 0);
  } else {
    ASSERT_EQ(dut.get_num_output_ports(), 1);
    const auto& descriptor = dut.get_output_port();
    EXPECT_EQ(descriptor.get_data_type(), kVectorValued);
    EXPECT_EQ(descriptor.size(), size);
  }
}

Eigen::VectorXd EvalOutput(
    const SymbolicVectorSystem& dut, const Context<double>& context) {
  const auto& output = dut.get_output_port().Allocate();
  dut.get_output_port().Calc(context, output.get());
  return output->GetValueOrThrow<BasicVector<double>>().CopyToVector();
}

Eigen::VectorXd EvalDerivatives(
    const SymbolicVectorSystem& dut, const Context<double>& context) {
  const auto& derivatives = dut.AllocateTimeDerivatives();
  dut.CalcTimeDerivatives(context, derivatives.get());
  return derivatives->CopyToVector();
}

#if 0
Eigen::VectorXd EvalDiscreteVariableUpdates(
    const SymbolicVectorSystem& dut, const Context<double>& context) {
  const auto& discrete_state = dut.AllocateDiscreteVariables();
  dut.CalcDiscreteVariableUpdates(context, discrete_state.get());
  return discrete_state->get_vector().CopyToVector();
}
#endif

#if 0
GTEST_TEST(SymbolicVectorSystemTest, ApiExample) {
  // This the sample code that appears in our constructor API docs.
  // We'll just make sure that it compiles.
  SymbolicVectorSystem gain(
      InputSize(3),
      OutputSize(3), [](const auto& arg) {
        arg.output = 2.0 * arg.input;
      });
}
#endif

GTEST_TEST(SymbolicVectorSystemTest, TypedSource) {
  // Output is a constant.
  const double value = 3.14;
  SymbolicVectorSystem dut(
      VectorSystemBlueprint::Make().
      WithOutput<test::Vec1>([=](const auto& arg) {
        arg.output.set_alpha(value);
      }));

  // Check the shape.
  CheckShape(dut, ParameterSize(0));
  CheckShape(dut, InputSize(0));
  CheckShape(dut, ContinuousStateSize(0));
  CheckShape(dut, DiscreteStateSize(0));
  CheckShape(dut, OutputSize(1));

  // Check the output.
  const auto& context = dut.CreateDefaultContext();
  EXPECT_EQ(EvalOutput(dut, *context), V1d(value));
}

GTEST_TEST(SymbolicVectorSystemTest, TypedGain) {
  // Output is a constant factor times the input.
  SymbolicVectorSystem dut(
      VectorSystemBlueprint::Make().
      WithInput<test::Vec1>().
      WithOutput<test::Vec2>([=](const auto& arg) {
        arg.output.set_beta(arg.input.alpha() * 0.1);
        arg.output.set_gamma(-1.0);
      }));

  // Check the shape.
  CheckShape(dut, ParameterSize(0));
  CheckShape(dut, InputSize(1));
  CheckShape(dut, ContinuousStateSize(0));
  CheckShape(dut, DiscreteStateSize(0));
  CheckShape(dut, OutputSize(2));

  // Check the output.
  const auto& context = dut.CreateDefaultContext();
  test::Vec1<double> input;
  input.set_alpha(4.0);
  context->FixInputPort(0, input);
  EXPECT_TRUE(CompareMatrices(EvalOutput(dut, *context), V2d(0.4, -1.0)));
}

GTEST_TEST(SymbolicVectorSystemTest, SecondOrderSystem) {
  // Input is acceleration, output is position.
  const SymbolicVectorSystem dut(
      VectorSystemBlueprint::Make().
      WithInput<test::Vec3>().
      WithContinuousState<test::Vec2>([=](const auto& arg) {
        arg.derivatives.set_beta(arg.input.x());
        arg.derivatives.set_gamma(arg.derivatives.beta());
      }).
      WithOutput<test::Vec1>([=](const auto& arg) {
        arg.output.set_alpha(arg.state.gamma());
      }));

  // Check the shape.
  CheckShape(dut, ParameterSize(0));
  CheckShape(dut, InputSize(3));
  CheckShape(dut, ContinuousStateSize(2));
  CheckShape(dut, DiscreteStateSize(0));
  CheckShape(dut, OutputSize(1));

  // Check the derivatives and output.
  const auto& context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state_vector().SetFromVector(V2d(1., 2.));
  context->FixInputPort(0, V1d(3.));
  EXPECT_EQ(EvalDerivatives(dut, *context), V2d(2., 3.));
  EXPECT_EQ(EvalOutput(dut, *context), V1d(1.));
}

#if 0
GTEST_TEST(SymbolicVectorSystemTest, SimpleContinuousSystem) {
  // Simple Continuous Time System: xdot = -x + x^3; y = x.
  const SymbolicVectorSystem dut(
      ContinuousStateSize(1), [](auto& arg) {
        using std::pow;
        arg.derivatives[0] = -arg.state[0] + pow(arg.state[0], 3.);
      },
      OutputSize(1), [](auto& arg) {
        arg.output = arg.state;
      });

  // Check the shape.
  CheckShape(dut, ParameterSize(0));
  CheckShape(dut, InputSize(0));
  CheckShape(dut, ContinuousStateSize(1));
  CheckShape(dut, DiscreteStateSize(0));
  CheckShape(dut, OutputSize(1));

  // Check the derivatives and output.
  const auto& context = dut.CreateDefaultContext();
  context->get_mutable_continuous_state_vector().SetFromVector(V1d(2.));
  EXPECT_EQ(EvalDerivatives(dut, *context), V1d(6.));
  EXPECT_EQ(EvalOutput(dut, *context), V1d(2.));
}

GTEST_TEST(SymbolicVectorSystemTest, SimpleDiscreteSystem) {
  // Simple Discrete Time System: x[n+1] = x[n]^3
  const SymbolicVectorSystem dut(
      DiscreteStateSize(1), DiscreteUpdatePeriod{1.0}, [](auto& arg) {
        using std::pow;
        arg.next_state[0] = pow(arg.state[0], 3.);
      },
      OutputSize(1), [](auto& arg) {
        arg.output = arg.state;
      });

  // Check the shape.
  CheckShape(dut, ParameterSize(0));
  CheckShape(dut, InputSize(0));
  CheckShape(dut, ContinuousStateSize(0));
  CheckShape(dut, DiscreteStateSize(1));
  CheckShape(dut, OutputSize(1));

  // Check the derivatives and output.
  const auto& context = dut.CreateDefaultContext();
  context->get_mutable_discrete_state(0).SetFromVector(V1d(2.));
  EXPECT_EQ(EvalDiscreteVariableUpdates(dut, *context), V1d(8.));
  EXPECT_EQ(EvalOutput(dut, *context), V1d(2.));
}
#endif

GTEST_TEST(SymbolicVectorSystemTest, TypedOutput2) {
  SymbolicVectorSystem dut(
      VectorSystemBlueprint::Make().
#if 0
      WithInput<Sample>().
      WithContinuousState<Sample>([](const auto& arg) {
        arg.derivatives.set_x(0.5 * arg.input.x());
      }).
#endif
      WithOutput<test::Vec1>([](const auto& arg) {
        arg.output.set_alpha(2.0);
      }));

  // Check the shape.
  CheckShape(dut, ParameterSize(0));
  CheckShape(dut, InputSize(0));
  CheckShape(dut, ContinuousStateSize(0));
  CheckShape(dut, DiscreteStateSize(0));
  CheckShape(dut, OutputSize(1));

  // Check the derivatives and output.
  const auto& context = dut.CreateDefaultContext();
  EXPECT_EQ(EvalOutput(dut, *context), V1d(2.));
}

// XXX check sparsity
// XXX cover all of the overloads
// XXX make sure it transmogs!

}  // namespace
}  // namespace systems
}  // namespace drake
