#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace systems {
namespace lcm {

/** On any per-step event where the request message input port has changed its
value, publishes the reply message.

@system
name: LcmRequestReply
input_ports:
- request
- reply
@endsystem

@ingroup message_passing */
class LcmRequestReply final : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmRequestReply)

  /** A factory method that creates an %LcmRequestReply system.

  @tparam RequestMessage the message type of the `request` input port.
  @tparam ReplyMessage the message type of the `reply` input port.

  @param reply_channel The LCM channel on which to publish.

  @param lcm A pointer to the LCM subsystem to use, which must remain valid for
  the lifetime of this object. Must be non-null. */
  template <typename RequestMessage, typename ReplyMessage>
  static std::unique_ptr<LcmRequestReply> Make(
      const std::string& reply_channel,
      drake::lcm::DrakeLcmInterface* lcm);
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
