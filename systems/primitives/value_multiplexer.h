#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** %ValueMultiplexer combines multiple abstract-valued input ports into a
single abstract-valued output port.  The input to this system directly feeds
through to its output.

@system{ValueMultiplexer, 
  @input_port{input0}
  @input_port{...},
  @input_port{inputN-1},
  @output_port{output}
}

The abstract values cannot be scalar-dependent.  The same conversion function
is used for all scalar types.

@tparam_default_scalar
@ingroup primitive_systems
*/
template <typename T>
class ValueMultiplexer final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ValueMultiplexer)

  /** The signature of the user-supplied multiplexing function. */
  using CalcFunction = std::function<void(  // NOLINT(whitespace/operators)
      const std::vector<const AbstractValue*>& /* inputs */,
      AbstractValue* /* output */)>;

  /** Constructs a %ValueMultiplexer with `num_inputs` input ports.  The value
  type of all input ports is given by `model_input`.  The value type of the
  sole output port is given by `model_output`.  The `calc` function specifies
  how to combine the inputs into the output. */
  explicit ValueMultiplexer(
      int num_inputs, const AbstractValue& model_input,
      const AbstractValue& model_output, CalcFunction calc);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit ValueMultiplexer(const ValueMultiplexer<U>&);

 private:
  template <typename> friend class ValueMultiplexer;

  const std::unique_ptr<const AbstractValue> model_input_;
  const std::unique_ptr<const AbstractValue> model_output_;
  const CalcFunction calc_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ValueMultiplexer)
