/* clang-format off */
#include "drake/lcm/drake_lcm.h"
/* clang-format on */

#include <source_location>
#include <stdexcept>

#include <fmt/format.h>

namespace drake {
namespace lcm {
namespace {
[[noreturn]] void ThrowError(
    std::source_location caller = std::source_location::current()) {
  throw std::logic_error(fmt::format(
      "{} cannot be used because the LCM runtime library has been disabled "
      "in this build of Drake",
      caller.function_name()));
}
}  // namespace

class DrakeLcm::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl);
};

DrakeLcm::DrakeLcm() = default;

DrakeLcm::DrakeLcm(std::string) {}

DrakeLcm::DrakeLcm(const DrakeLcmParams&) {}

DrakeLcm::~DrakeLcm() = default;

std::string DrakeLcm::get_lcm_url() const {
  return "unavailable://";
}

void DrakeLcm::Publish(const std::string&, const void*, int,
                       std::optional<double>) {
  ThrowError();
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcm::Subscribe(
    const std::string&, HandlerFunction) {
  return nullptr;
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcm::SubscribeMultichannel(
    std::string_view, MultichannelHandlerFunction) {
  return nullptr;
}

std::shared_ptr<DrakeSubscriptionInterface> DrakeLcm::SubscribeAllChannels(
    MultichannelHandlerFunction) {
  return nullptr;
}

int DrakeLcm::HandleSubscriptions(int) {
  ThrowError();
}

void* DrakeLcm::get_native_lcm_handle_for_unit_testing() {
  ThrowError();
}

void DrakeLcm::OnHandleSubscriptionsError(const std::string&) {
  ThrowError();
}

bool DrakeLcm::available() {
  return false;
}

}  // namespace lcm
}  // namespace drake
