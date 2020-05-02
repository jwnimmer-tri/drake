#include "drake/systems/primitives/value_multiplexer.h"

#include <utility>

namespace drake {
namespace systems {

template <typename T>
ValueMultiplexer<T>::ValueMultiplexer(
      int num_inputs, const AbstractValue& model_input,
      const AbstractValue& model_output, CalcFunction calc)
    : LeafSystem<T>(SystemTypeTag<ValueMultiplexer>{}),
      model_input_(model_input.Clone()),
      model_output_(model_output.Clone()),
      calc_(std::move(calc)) {
  DRAKE_THROW_UNLESS(num_inputs >= 0);
  DRAKE_THROW_UNLESS(bool{calc_});
  for (int i = 0; i < num_inputs; ++i) {
    this->DeclareAbstractInputPort(fmt::format("input", i), model_input);
  }
  auto alloc_output =
      [this]() {
        return this->model_output_->Clone();
      };
  auto calc_output =
      [this, inputs = std::vector<const AbstractValue*>(num_inputs)](
           const Context<T>& context, AbstractValue* output) mutable {
        for (size_t i = 0; i < inputs.size(); ++i) {
          const auto& port = this->get_input_port(i);
          inputs[i] = &(port.template Eval<AbstractValue>(context));
        }
        this->calc_(inputs, output);
      };
  this->DeclareAbstractOutputPort(
      "output", std::move(alloc_output), std::move(calc_output));
}

template <typename T>
template <typename U>
ValueMultiplexer<T>::ValueMultiplexer(const ValueMultiplexer<U>& other)
    : ValueMultiplexer<T>(
          other.num_input_ports(), *(other.model_input_->Clone()),
          *(other.model_output_->Clone()), other.calc_) {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ValueMultiplexer)
