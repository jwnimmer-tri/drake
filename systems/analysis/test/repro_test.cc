#include <string>

#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::lcm::Subscriber;
using drake::systems::lcm::LcmInterfaceSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::lcmt_drake_signal;

namespace drake {
namespace systems {
namespace {

class PeriodicEventSystem : public LeafSystem<double> {
 public:
  PeriodicEventSystem(double period = 0.25) {
    this->DeclarePeriodicPublishEvent(
        period, 0., &PeriodicEventSystem::MakeItCount);
  }

  int publish_count() const { return publish_counter_; }
  void reset_count() { publish_counter_ = 0; }

 private:
  EventStatus MakeItCount(const Context<double>&) const {
    ++publish_counter_;
    return EventStatus::Succeeded();
  }
  mutable int publish_counter_{0};
};

class BoolReproTest : public ::testing::TestWithParam<bool> {};

TEST_P(BoolReproTest, AcceptanceTest) {
  const bool use_simulator_to_process_lcm_messages = GetParam();

  // Prepare a simple diagram ...
  DiagramBuilder<double> builder;

  // LCM will happen via the memq URL, which does not use the network at all.
  // Depending on the test parameter, either the Simulator do the event
  // processing, or else we'll do it manually within our do-while loop below.
  DrakeLcm memq("memq://");
  DrakeLcmInterface* const memq_interface = &memq;
  DrakeLcmInterface* const lcm =
      use_simulator_to_process_lcm_messages
      ? builder.AddSystem<LcmInterfaceSystem>(&memq)
      : memq_interface;

  // Add a per-step LCM publication.
  auto* source = builder.AddSystem<ConstantValueSource<double>>(
      Value<lcmt_drake_signal>(lcmt_drake_signal{}));
  auto* sink = builder.AddSystem(LcmPublisherSystem::Make<lcmt_drake_signal>(
      "CHANNEL", lcm));
  builder.Cascade(*source, *sink);

  // Subscribe to that LCM channel, so that memq does not discard the messages.
  Subscriber<lcmt_drake_signal> lcm_per_step(&memq, "CHANNEL");

  // Separately, declate a periodic timed event.
  auto* publish_60hz = builder.AddSystem<PeriodicEventSystem>(1.0 / 60.0);

  // Simulate, while processing LCM.
  Simulator<double> simulator(builder.Build());
  simulator.Initialize();
  do {
    if (!use_simulator_to_process_lcm_messages) {
      while (lcm->HandleSubscriptions(0) > 0) {}
    }
    simulator.AdvanceTo(simulator.get_context().get_time() + 0.005);
  } while (simulator.get_context().get_time() < 10.0);

  // Check that we got the correct counts.
  // The per step events should be 200..260 Hz.
  EXPECT_GE(lcm_per_step.count(), 200 * 10);
  EXPECT_LE(lcm_per_step.count(), 260 * 10);
  // The timed events should be exactly 60 Hz.
  EXPECT_EQ(publish_60hz->publish_count(), 60 * 10 + 1);
}

INSTANTIATE_TEST_SUITE_P(
    BothOptions, BoolReproTest,
    ::testing::ValuesIn({true, false}));

}  // namespace
}  // namespace systems
}  // namespace drake
