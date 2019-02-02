#include "drake/systems/framework/symbolic_vector_system.h"

#include <string>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/system_type_tag.h"

// Don't let Doxygen pick up on any hidden details.
#if !defined(DRAKE_DOXYGEN_CXX)

using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::symbolic::Variables;

namespace drake {
namespace systems {
namespace {

#if 0
// XXX Make this a common helper.
auto MakeVariables(const char* prefix, int size) {
  VectorX<Expression> result(size);
  for (int i = 0; i < size; ++i) {
    std::ostringstream name;
    name << prefix << i;
    result[i] = Variable(name.str());
  }
  return result;
}
#endif

#if 0
// XXX Make this a common helper.
Variables UnionOfVectorExpressionVariables(
    const VectorX<Expression>& expressions) {
  Variables result;
  for (int i = 0; i < expressions.size(); ++i) {
    result += expressions[i].GetVariables();
  }
  return result;
}
#endif

#if 0
struct SymbolicContext {
  // Disable copy and move, because we keep internal pointers to ourself.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SymbolicContext)

  // The storage for all of our variables (in expression form).
  const Expression time_storage;
  const VectorX<Expression> input_storage;
  const VectorX<Expression> state_storage;
  const VectorX<Expression> parameter_storage;

  // The unordered sets of all of our variables (for fast intersection).
  const Variable time_var;
  const Variables input_vars;
  const Variables state_vars;
  const Variables parameter_vars;

  // The references that we will pass to Calc functions.
  using ConstBlock = Eigen::VectorBlock<const VectorX<Expression>>;
  const Expression& time;
  const ConstBlock input;
  const ConstBlock state;
  const ConstBlock parameter;

  SymbolicContext(
      InputSize input_size,
      ContinuousStateSize continuous_state_size,
      DiscreteStateSize discrete_state_size,
      ParameterSize parameter_size)
      : time_storage(Variable("t")),
        input_storage(MakeVariables("u", input_size)),
        state_storage(
            MakeVariables("x", continuous_state_size + discrete_state_size)),
        parameter_storage(MakeVariables("p", parameter_size)),
        time_var(get_variable(time_storage)),
        input_vars(UnionOfVectorExpressionVariables(input_storage)),
        state_vars(UnionOfVectorExpressionVariables(state_storage)),
        parameter_vars(UnionOfVectorExpressionVariables(parameter_storage)),
        time(time_storage),
        input(input_storage.segment(0, input_size)),
        state(state_storage.segment(0, state_storage.size())),
        parameter(parameter_storage.segment(0, parameter_size)) {}
};
#endif

#if 0
struct SymbolicDependencies {
  SymbolicDependencies() = default;

  explicit SymbolicDependencies(
      const SymbolicContext& symbolic_context,
      const VectorX<Expression>& result_of_calc) {
    const Variables vars = UnionOfVectorExpressionVariables(result_of_calc);
    uses_time =  vars.include(symbolic_context.time_var);
    uses_parameter = !intersect(vars, symbolic_context.parameter_vars).empty();
    uses_input = !intersect(vars, symbolic_context.input_vars).empty();
    uses_state = !intersect(vars, symbolic_context.state_vars).empty();
    const int result_size = result_of_calc.size();
    expressions.reserve(result_size);
    for (int i = 0; i < result_size; ++i) {
      expressions.emplace_back(result_of_calc[i].to_string());
    }
  }

  bool uses_time{false};
  bool uses_parameter{false};
  bool uses_input{false};
  bool uses_state{false};
  std::vector<std::string> expressions;
};
#endif

#if 0
SymbolicDependencies GetOutputDependencies(
    const SymbolicContext& symbolic_context,
    OutputSize output_size,
    const CalcOutputLambdas& lambdas) {
  if (output_size == 0) {
    return {};
  }

  // Symbolically evaluate the lambda.
  VectorX<Expression> output_storage = MakeVariables("y", output_size);
  auto output = output_storage.segment(0, output_size);
  const ArgsForCalcOutput<Expression> args(
      symbolic_context.time,
      symbolic_context.input,
      symbolic_context.state,
      symbolic_context.parameter,
      &output);
  lambdas.Call(args);

  // Populate the result.
  return SymbolicDependencies(symbolic_context, output_storage);
}

SymbolicDependencies GetDerivativesDependencies(
    const SymbolicContext& symbolic_context,
    const CalcDerivativesLambdas& lambdas) {
  const int state_size = symbolic_context.state.size();
  DRAKE_DEMAND(state_size > 0);

  // Symbolically evaluate the lambda.
  VectorX<Expression> derivatives_storage =
      MakeVariables("xdot", state_size);
  auto derivatives = derivatives_storage.segment(0, state_size);
  const ArgsForCalcDerivatives<Expression> args(
      symbolic_context.time,
      symbolic_context.input,
      symbolic_context.state,
      symbolic_context.parameter,
      &derivatives);
  lambdas.Call(args);

  // Populate the result.
  return SymbolicDependencies(symbolic_context, derivatives_storage);
}

SymbolicDependencies GetNextStateDependencies(
    const SymbolicContext& symbolic_context,
    const CalcNextStateLambdas& lambdas) {
  const int state_size = symbolic_context.state.size();
  DRAKE_DEMAND(state_size > 0);

  // Symbolically evaluate the lambda.
  VectorX<Expression> next_state_storage =
      MakeVariables("xnext", state_size);
  auto next_state = next_state_storage.segment(0, state_size);
  const ArgsForCalcNextState<Expression> args(
      symbolic_context.time,
      symbolic_context.input,
      symbolic_context.state,
      symbolic_context.parameter,
      &next_state);
  lambdas.Call(args);

  // Populate the result.
  return SymbolicDependencies(symbolic_context, next_state_storage);
}
#endif

}  // namespace

template <typename T>
SymbolicVectorSystemBase<T>::SymbolicVectorSystemBase(
    VectorSystemBlueprint blueprint)
    : SymbolicVectorSystemBase(
          std::make_shared<const VectorSystemBlueprint>(
              std::move(blueprint))) {}

// All construction work interacting with LeafSystem (our superclass) should
// happen here -- all other constructors delegate to this one.
template <typename T>
SymbolicVectorSystemBase<T>::SymbolicVectorSystemBase(
    const std::shared_ptr<const VectorSystemBlueprint>& blueprint)
    : LeafSystem<T>(SystemTypeTag<systems::SymbolicVectorSystemBase>{}),
      blueprint_(blueprint) {
  // blueprint_->DeclareNumericParameterFor(this);
  blueprint_->DeclareContinuousStateFor(this);
  // blueprint_->DeclareDiscreteStateFor(this);
  // this->DeclarePeriodicDiscreteUpdate();
  blueprint_->DeclareInputFor(this);
  blueprint_->DeclareOutputFor(this);
}

template <typename T>
SymbolicVectorSystemBase<T>::~SymbolicVectorSystemBase() {}

template <typename T>
void SymbolicVectorSystemBase<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  blueprint_->CalcTimeDerivatives(*this, context, derivatives);
}

template <typename T>
void SymbolicVectorSystemBase<T>::DoCalcDiscreteVariableUpdates(
    const Context<T>& context,
    const std::vector<const DiscreteUpdateEvent<T>*>&,
    DiscreteValues<T>* discrete_state) const {
#if 0
  // An empty vector whose address is unchanging and always valid.
  static const never_destroyed<VectorX<T>> empty_vector(0);
  const auto& empty_block = empty_vector.access().segment(0, 0);

  // Obtain the aliases that we're going to supply into the calc call, below.
  // Compute only the necessary blocks; leave the rest empty.
  const T& time = context.get_time();
  const Eigen::VectorBlock<const VectorX<T>> parameter_block = [&]() {
    if (impl_->next_state_dependencies.uses_parameter) {
      return context.get_numeric_parameter(0).get_value();
    }
    return empty_block;
  }();
  const Eigen::VectorBlock<const VectorX<T>> input_block = [&]() {
    if (impl_->next_state_dependencies.uses_input) {
      return this->EvalEigenVectorInput(context, 0);
    }
    return empty_block;
  }();
  const Eigen::VectorBlock<const VectorX<T>> state_block = [&]() {
    if (impl_->next_state_dependencies.uses_state) {
      return context.get_discrete_state(0).get_value();
    }
    return empty_block;
  }();
  Eigen::VectorBlock<VectorX<T>> next_state_block = [&]() {
    return discrete_state->get_mutable_vector().get_mutable_value();
  }();

  // Call the lambda supplied to the SymbolicVectorSystem constructor.
  const ArgsForCalcNextState<T> args(
      time,
      input_block,
      state_block,
      parameter_block,
      &next_state_block);
  impl_->calc_next_state.Call(args);
#endif
}

}  // namespace systems
}  // namespace drake

#endif  // DRAKE_DOXYGEN_CXX

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SymbolicVectorSystemBase)
