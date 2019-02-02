#pragma once

#include <functional>
#include <memory>
#include <type_traits>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/null_vector.h"

namespace drake {
namespace systems {

class VectorSystemBlueprint;

template <typename T>
class SymbolicVectorSystemBase;

namespace vector_system_detail {

template <
  template <typename> class InputBasicVector,
  template <typename> class InputBasicVectorForCalcOutput,
  template <typename> class ContinuousStateBasicVector,
  typename CalcDerivativesGenericLambda,
  template <typename> class DiscreteStateBasicVector,
  typename CalcNextStateGenericLambda,
  template <typename> class OutputBasicVector,
  typename CalcOutputGenericLambda,
  template <typename> class ParameterBasicVector>
class TypedBlueprint final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TypedBlueprint)

  TypedBlueprint() = default;

  operator VectorSystemBlueprint() const;

  template <template <typename> class NewInputBasicVector>
  auto WithInput() {
    return vector_system_detail::TypedBlueprint<
      NewInputBasicVector,
      NewInputBasicVector,
      ContinuousStateBasicVector,
      CalcDerivativesGenericLambda,
      DiscreteStateBasicVector,
      CalcNextStateGenericLambda,
      OutputBasicVector,
      CalcOutputGenericLambda,
      ParameterBasicVector>{
        calc_derivatives_, calc_next_state_, calc_output_};
  }

  template <
    template <typename> class NewContinuousStateBasicVector,
    typename NewCalcDerivativesGenericLambda>
  auto WithContinuousState(NewCalcDerivativesGenericLambda lambda) const {
    return vector_system_detail::TypedBlueprint<
      InputBasicVector,
      InputBasicVectorForCalcOutput,
      NewContinuousStateBasicVector,
      NewCalcDerivativesGenericLambda,
      DiscreteStateBasicVector,
      CalcNextStateGenericLambda,
      OutputBasicVector,
      CalcOutputGenericLambda,
      ParameterBasicVector>{
        lambda, calc_next_state_, calc_output_};
  }

  template <
    template <typename> class NewOutputBasicVector,
    typename NewCalcOutputGenericLambda>
  auto WithOutput(NewCalcOutputGenericLambda lambda) const {
    return TypedBlueprint<
      InputBasicVector,
      InputBasicVectorForCalcOutput,
      ContinuousStateBasicVector,
      CalcDerivativesGenericLambda,
      DiscreteStateBasicVector,
      CalcNextStateGenericLambda,
      NewOutputBasicVector,
      NewCalcOutputGenericLambda,
      ParameterBasicVector>{
        calc_derivatives_, calc_next_state_, lambda};
  }

 private:
  // Be friends with other instantiations of ourself.
  template <
    template <typename> class,
    template <typename> class,
    template <typename> class,
    typename,
    template <typename> class,
    typename,
    template <typename> class,
    typename,
    template <typename> class>
  friend class TypedBlueprint;

  // Be friends with the SymbolicVectorSystem.
  template <typename>
  friend class systems::SymbolicVectorSystemBase;

  template <typename T>
  using UseInputBasicVector = InputBasicVector<T>;
  template <typename T>
  using UseContinuousStateBasicVector = ContinuousStateBasicVector<T>;
  template <typename T>
  using UseOutputBasicVector = OutputBasicVector<T>;

  using NoContinuousState =
      std::is_same<ContinuousStateBasicVector<double>, NullVector<double>>;
  using NoDiscreteState =
      std::is_same<DiscreteStateBasicVector<double>, NullVector<double>>;

  // If only continuous xor discrete state is declared, then this is an alias
  // for its vector type; otherwise, it is the null vector type.
  template <typename T>
  using StateBasicVector =
      typename std::conditional<
        NoDiscreteState::value,
        ContinuousStateBasicVector<T>,
        typename std::conditional<
          NoContinuousState::value,
          DiscreteStateBasicVector<T>,
          NullVector<T>>::type>::type;

  // This is more or less a copy constructor for substituting the BasicVector-
  // related template arguments.  We just have to copy the generic lambdas from
  // the other blueprint to ourself.
  TypedBlueprint(
      CalcDerivativesGenericLambda calc_derivatives,
      CalcNextStateGenericLambda calc_next_state,
      CalcOutputGenericLambda calc_output)
      : calc_derivatives_(calc_derivatives),
        calc_next_state_(calc_next_state),
        calc_output_(calc_output) {}

  // Helper to evaluate the input during some Calc.  The SomeBasicVector is the
  // type of the input (some BasicVectorSubclass) -- setting it to NullVector
  // will short-circuit the evaluation (return immediately).
  template <typename T, template <typename> class SomeBasicVector>
  const SomeBasicVector<T>& EvalInput(
      const System<T>& system,
      const Context<T>& context) const {
    return EvalInput<T, SomeBasicVector>(
        system, context,
        std::is_same<SomeBasicVector<T>, NullVector<T>>{});
  }
  template <typename T, template <typename> class SomeBasicVector>
  const SomeBasicVector<T>& EvalInput(
      const System<T>& system,
      const Context<T>& context,
      std::true_type is_null_vector) const {
    // Don't evaluate unwanted inputs.
    return NullVector<T>::get();
  }
  template <typename T, template <typename> class SomeBasicVector>
  const SomeBasicVector<T>& EvalInput(
      const System<T>& system,
      const Context<T>& context,
      std::false_type is_null_vector) const {
    const auto* const result =
        system.template EvalVectorInput<SomeBasicVector>(context, 0);
    // XXX Provide for a better error message when the input was not connected.
    DRAKE_THROW_UNLESS(result);
    return *result;
  }

  // Helper to evaluate the state during some Calc.
  template <typename T>
  const ContinuousStateBasicVector<T>& GetContinuousState(
      const Context<T>& context) const {
    if (std::is_same<ContinuousStateBasicVector<T>, NullVector<T>>::value) {
      return dynamic_cast<const ContinuousStateBasicVector<T>&>(NullVector<T>::get());
    }
    const VectorBase<T>& result = context.get_continuous_state_vector();
    return dynamic_cast<const ContinuousStateBasicVector<T>&>(result);
  }

  // Helper to evaluate the state during some Calc.
  template <typename T>
  const DiscreteStateBasicVector<T>& GetDiscreteState(const Context<T>& context) const {
    if (std::is_same<DiscreteStateBasicVector<T>, NullVector<T>>::value) {
      return NullVector<T>::get();
    }
    const VectorBase<T>& result = context.get_discrete_state_vector();
    return dynamic_cast<const DiscreteStateBasicVector<T>&>(result);
  }

  // Helper to evaluate the state during some Calc.
  template <typename T>
  const StateBasicVector<T>& AliasState(
      const ContinuousStateBasicVector<T>& cont,
      const DiscreteStateBasicVector<T>& disc) const {
    if (NoContinuousState::value) {
      return dynamic_cast<const StateBasicVector<T>&>(disc);
    }
    return cont;
  }

  // Helper to evaluate the parameter during some Calc.
  template <typename T>
  const ParameterBasicVector<T>& GetParameter(const Context<T>& context) const {
    if (std::is_same<ParameterBasicVector<T>, NullVector<T>>::value) {
      return NullVector<T>::get();
    }
    const BasicVector<T>& result = context.get_numeric_parameter(0);
    return dynamic_cast<const ParameterBasicVector<T>&>(result);
  }

  // The structure passed to the user's CalcOutput lambda as its sole argument.
  template <typename T>
  struct ArgsForCalcOutput {
    const T& time;
    const InputBasicVectorForCalcOutput<T>& input;
    const StateBasicVector<T>& state;
    const ContinuousStateBasicVector<T>& continuous_state;
    const DiscreteStateBasicVector<T>& discrete_state;
    const ParameterBasicVector<T>& parameter;
    OutputBasicVector<T>& output;
  };

  template <typename T>
  void CalcOutput(
      const System<T>& system,
      const Context<T>& context,
      BasicVector<T>* basic_output) const {
    const T time = context.get_time();
    const auto& input = EvalInput<T, InputBasicVectorForCalcOutput>(system, context);
    const auto& continuous_state = GetContinuousState(context);
    const auto& discrete_state = GetDiscreteState(context);
    const auto& state = AliasState(continuous_state, discrete_state);
    const auto& parameter = GetParameter(context);
    auto* output = dynamic_cast<OutputBasicVector<T>*>(basic_output);
    DRAKE_DEMAND(output != nullptr);
    ArgsForCalcOutput<T> args{
        time, input, state, continuous_state, discrete_state, parameter, *output};
    calc_output_(args);
  }

  // Return a functor that (when given a System pointer) declares this
  // blueprint's specified input port.
  template <typename T>
  auto MakeDeclareInput() const {
    return [self = *this](SymbolicVectorSystemBase<T>* system) {
      if (!std::is_same<InputBasicVector<T>, NullVector<T>>::value) {
        system->DeclareBlueprintInput(self);
      }
    };
  }

  // Return a functor that (when given a System pointer) declares this
  // blueprint's specified continuous state.
  template <typename T>
  auto MakeDeclareContinuousState() const {
    return [self = *this](SymbolicVectorSystemBase<T>* system) {
      if (!std::is_same<ContinuousStateBasicVector<T>, NullVector<T>>::value) {
        system->DeclareBlueprintContinuousState(self);
      }
    };
  }

  // Return a functor that (when given a System pointer) declares this
  // blueprint's specified output port.
  template <typename T>
  auto MakeDeclareOutput() const {
    return [self = *this](SymbolicVectorSystemBase<T>* system) {
      if (!std::is_same<OutputBasicVector<T>, NullVector<T>>::value) {
        system->DeclareBlueprintOutput(self);
      }
    };
  }

  // Return a functor that ...
  template <typename T>
  auto MakeCalcTimeDerivatives() const {
    return [self = *this](
        const System<T>& system,
        const Context<T>& context,
        ContinuousState<T>* derivatives) {
      // XXX
#if 0
      calc_derivatives_(args);
#endif
    };
  }

  CalcDerivativesGenericLambda calc_derivatives_;
  CalcNextStateGenericLambda calc_next_state_;
  CalcOutputGenericLambda calc_output_;
};

}  // namespace vector_system_detail

/// XXX
class VectorSystemBlueprint final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VectorSystemBlueprint)
  VectorSystemBlueprint() = default;

  using Expression = symbolic::Expression;

  static auto Make() {
    return vector_system_detail::TypedBlueprint<
      NullVector,  // input
      NullVector,  // input for CalcOutput
      NullVector,  // continuous state
      std::function<void()>,
      NullVector,  // discrete state
      std::function<void()>,
      NullVector,  // output
      std::function<void()>,
      NullVector>{};  // parameter
  }

 private:
  // Be friends with ... XXX.
  template <
    template <typename> class,
    template <typename> class,
    template <typename> class,
    typename,
    template <typename> class,
    typename,
    template <typename> class,
    typename,
    template <typename> class>
  friend class vector_system_detail::TypedBlueprint;

  // Be friends with the SymbolicVectorSystem.
  template <typename>
  friend class systems::SymbolicVectorSystemBase;

  // XXX
  void DeclareInputFor(SymbolicVectorSystemBase<double>* sys) const {
    double_impl_.declare_input(sys);
  }
  void DeclareInputFor(SymbolicVectorSystemBase<AutoDiffXd>* sys) const {
    autodiffxd_impl_.declare_input(sys);
  }
  void DeclareInputFor(SymbolicVectorSystemBase<Expression>* sys) const {
    symbolic_impl_.declare_input(sys);
  }

  // XXX
  void DeclareContinuousStateFor(SymbolicVectorSystemBase<double>* sys) const {
    double_impl_.declare_continuous_state(sys);
  }
  void DeclareContinuousStateFor(SymbolicVectorSystemBase<AutoDiffXd>* sys) const {
    autodiffxd_impl_.declare_continuous_state(sys);
  }
  void DeclareContinuousStateFor(SymbolicVectorSystemBase<Expression>* sys) const {
    symbolic_impl_.declare_continuous_state(sys);
  }

  // XXX
  void DeclareOutputFor(SymbolicVectorSystemBase<double>* sys) const {
    double_impl_.declare_output(sys);
  }
  void DeclareOutputFor(SymbolicVectorSystemBase<AutoDiffXd>* sys) const {
    autodiffxd_impl_.declare_output(sys);
  }
  void DeclareOutputFor(SymbolicVectorSystemBase<Expression>* sys) const {
    symbolic_impl_.declare_output(sys);
  }

  // XXX
  void CalcTimeDerivatives(
      const System<double>& system,
      const Context<double>& context,
      ContinuousState<double>* derivatives) const {
    double_impl_.calc_time_derivatives(system, context, derivatives);
  }
  void CalcTimeDerivatives(
      const System<AutoDiffXd>& system,
      const Context<AutoDiffXd>& context,
      ContinuousState<AutoDiffXd>* derivatives) const {
    autodiffxd_impl_.calc_time_derivatives(system, context, derivatives);
  }
  void CalcTimeDerivatives(
      const System<Expression>& system,
      const Context<Expression>& context,
      ContinuousState<Expression>* derivatives) const {
    symbolic_impl_.calc_time_derivatives(system, context, derivatives);
  }

  template <typename T>
  struct Impl {
    using DeclareSystemAttributeFunction =
        std::function<void(SymbolicVectorSystemBase<T>*)>;
    using CalcTimeDerivativesFunction =
        std::function<void(const System<T>&, const Context<T>&,
                           ContinuousState<T>*)>;

    DeclareSystemAttributeFunction declare_output;
    DeclareSystemAttributeFunction declare_input;
    DeclareSystemAttributeFunction declare_continuous_state;
    CalcTimeDerivativesFunction calc_time_derivatives;
  };

  Impl<double> double_impl_;
  Impl<AutoDiffXd> autodiffxd_impl_;
  Impl<Expression> symbolic_impl_;
};

namespace vector_system_detail {

template <
  template <typename> class InputBasicVector,
  template <typename> class InputBasicVectorForCalcOutput,
  template <typename> class ContinuousStateBasicVector,
  typename CalcDerivativesGenericLambda,
  template <typename> class DiscreteStateBasicVector,
  typename CalcNextStateGenericLambda,
  template <typename> class OutputBasicVector,
  typename CalcOutputGenericLambda,
  template <typename> class ParameterBasicVector>
TypedBlueprint<
  InputBasicVector,
  InputBasicVectorForCalcOutput,
  ContinuousStateBasicVector,
  CalcDerivativesGenericLambda,
  DiscreteStateBasicVector,
  CalcNextStateGenericLambda,
  OutputBasicVector,
  CalcOutputGenericLambda,
  ParameterBasicVector>::
operator VectorSystemBlueprint() const {
  VectorSystemBlueprint result;

  result.double_impl_.declare_input = MakeDeclareInput<double>();
  result.autodiffxd_impl_.declare_input = MakeDeclareInput<AutoDiffXd>();
  result.symbolic_impl_.declare_input = MakeDeclareInput<symbolic::Expression>();

  result.double_impl_.declare_continuous_state = MakeDeclareContinuousState<double>();
  result.autodiffxd_impl_.declare_continuous_state = MakeDeclareContinuousState<AutoDiffXd>();
  result.symbolic_impl_.declare_continuous_state = MakeDeclareContinuousState<symbolic::Expression>();

  result.double_impl_.declare_output = MakeDeclareOutput<double>();
  result.autodiffxd_impl_.declare_output = MakeDeclareOutput<AutoDiffXd>();
  result.symbolic_impl_.declare_output = MakeDeclareOutput<symbolic::Expression>();

  result.double_impl_.calc_time_derivatives = MakeCalcTimeDerivatives<double>();
  result.autodiffxd_impl_.calc_time_derivatives = MakeCalcTimeDerivatives<AutoDiffXd>();
  result.symbolic_impl_.calc_time_derivatives = MakeCalcTimeDerivatives<symbolic::Expression>();

  return result;
}

}  // namespace vector_system_detail

/// XXX API documentation.
/// XXX A final class named Base is weird.
template <typename T>
class SymbolicVectorSystemBase final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SymbolicVectorSystemBase)

  /// Constructs a SymbolicVectorSystem ... XXX
  explicit SymbolicVectorSystemBase(VectorSystemBlueprint);

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit SymbolicVectorSystemBase(const SymbolicVectorSystemBase<U>& other)
      : SymbolicVectorSystemBase<T>(other.blueprint_) {}

  ~SymbolicVectorSystemBase() final;

  /// Returns the sole input port if one has been declared, else throws.
  const InputPortDescriptor<T>& get_input_port() const {
    DRAKE_THROW_UNLESS(this->get_num_input_ports() == 1);
    return LeafSystem<T>::get_input_port(0);
  }

  // Don't use the indexed get_input_port when calling this system directly.
  void get_input_port(int) = delete;

  /// Returns the sole output port if one has been declared, else throws.
  const OutputPort<T>& get_output_port() const {
    DRAKE_THROW_UNLESS(this->get_num_output_ports() == 1);
    return LeafSystem<T>::get_output_port(0);
  }

  // Don't use the indexed get_output_port when calling this system directly.
  void get_output_port(int) = delete;

 private:
  // Allow other `SymbolicVectorSystemBase<U>`'s to directly read `impl_`.
  template <typename> friend class SymbolicVectorSystemBase;

  // Be friends with all instantiations of blueprints, so that they can declare
  // outputs on our behalf.
  template <
    template <typename> class,
    template <typename> class,
    template <typename> class,
    typename,
    template <typename> class,
    typename,
    template <typename> class,
    typename,
    template <typename> class>
  friend class vector_system_detail::TypedBlueprint;

  // All other constructors delegate here.
  explicit SymbolicVectorSystemBase(
      const std::shared_ptr<const VectorSystemBlueprint>&);

  // XXX doc
  template <typename SpecificBlueprint>
  void DeclareBlueprintInput(const SpecificBlueprint& blueprint) {
    using ModelVector =
        typename SpecificBlueprint::template UseInputBasicVector<T>;
    this->DeclareVectorInputPort(ModelVector{});
  }

  // XXX doc
  template <typename SpecificBlueprint>
  void DeclareBlueprintContinuousState(const SpecificBlueprint& blueprint) {
    using ModelVector =
        typename SpecificBlueprint::template UseContinuousStateBasicVector<T>;
    this->DeclareContinuousState(ModelVector{});
    // XXX update function
  }

  // XXX doc
  template <typename SpecificBlueprint>
  void DeclareBlueprintOutput(const SpecificBlueprint& blueprint) {
    using ModelVector =
        typename SpecificBlueprint::template UseOutputBasicVector<T>;
    this->DeclareVectorOutputPort(
        ModelVector{},
        [this, blueprint](
            const Context<T>& context, BasicVector<T>* output) {
          blueprint.CalcOutput(*this, context, output);
        });
  }

  // Helpers.
  void DoCalcTimeDerivatives(
      const Context<T>&, ContinuousState<T>*) const final;

  void DoCalcDiscreteVariableUpdates(
      const Context<T>&, const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>*) const final;

  // As we transmogrify, all copies will share the same blueprint instance.
  const std::shared_ptr<const VectorSystemBlueprint> blueprint_;
};

/// XXX API Documentation.
using SymbolicVectorSystem = SymbolicVectorSystemBase<double>;

}  // namespace systems
}  // namespace drake
