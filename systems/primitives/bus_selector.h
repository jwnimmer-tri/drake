#pragma once

#include <string>
#include <variant>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** This system ...

@system
name: BusSelector
input_ports:
- u0
output_ports:
- y0
- ...
- y(N-1)
@endsystem

@tparam_default_scalar
@ingroup primitive_systems */
template <typename T>
class BusSelector final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BusSelector);

  /** Constructs a %BusSelector with ... */
  explicit BusSelector(std::variant<std::string, UseDefaultName>
                           input_port_name = kUseDefaultName);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit BusSelector(const BusSelector<U>&);

  ~BusSelector() final;

  OutputPort<T>& DeclareVectorOutputPort(
      std::variant<std::string, UseDefaultName> name, int size);

  OutputPort<T>& DeclareAbstractOutputPort(
      std::variant<std::string, UseDefaultName> name,
      const AbstractValue& model_value);

 private:
  template <typename>
  friend class BusSelector;

  void CalcVectorOutput(const Context<T>& context, OutputPortIndex index,
                        BasicVector<T>* output) const;

  void CalcAbstractOutput(const Context<T>& context, OutputPortIndex index,
                          AbstractValue* output) const;
};

}  // namespace systems
}  // namespace drake
