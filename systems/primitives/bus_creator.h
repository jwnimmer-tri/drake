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
name: BusCreator
input_ports:
- u0
- ...
- u(N-1)
output_ports:
- y0
@endsystem

@tparam_default_scalar
@ingroup primitive_systems */
template <typename T>
class BusCreator final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BusCreator);

  /** Constructs a %BusCreator with no inputs. Use DeclareAbstractInputPort()
  and DeclareVectorInputPort() to add ports. */
  explicit BusCreator(std::variant<std::string, UseDefaultName>
                          output_port_name = kUseDefaultName);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit BusCreator(const BusCreator<U>&);

  ~BusCreator() final;

  InputPort<T>& DeclareVectorInputPort(
      std::variant<std::string, UseDefaultName> name, int size);

  InputPort<T>& DeclareAbstractInputPort(
      std::variant<std::string, UseDefaultName> name,
      const AbstractValue& model_value);

 private:
  template <typename>
  friend class BusCreator;

  void CalcOutput(const Context<T>& context, BusValue* output) const;
};

}  // namespace systems
}  // namespace drake
