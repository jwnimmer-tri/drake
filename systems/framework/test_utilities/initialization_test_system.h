#pragma once

#include <vector>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** A LeafSystem which declares all possible initialization events, and records
 information (in mutable variables and in the context) about whether the events
 have been processed. This can be used to confirm that initialization events
 are properly handled. */
class InitializationTestSystem : public LeafSystem<double> {
 public:
  InitializationTestSystem() {
    PublishEvent<double> pub_event;
    pub_event.set_callback([](const System<double>& system,
                              const Context<double>& context,
                              const PublishEvent<double>& event) {
      dynamic_cast<const InitializationTestSystem&>(system).InitPublish(context,
                                                                        event);
      return EventStatus::Succeeded();
    });
    DeclareInitializationEvent(pub_event);

    DeclareDiscreteState(1);
    DeclareAbstractState(Value<bool>(false));

    DeclareInitializationEvent(DiscreteUpdateEvent<double>{});
    DeclareInitializationEvent(UnrestrictedUpdateEvent<double>{});
  }

  bool get_pub_init() const { return pub_init_; }
  bool get_dis_update_init() const { return dis_update_init_; }
  bool get_unres_update_init() const { return unres_update_init_; }

 private:
  void InitPublish(const Context<double>&,
                   const PublishEvent<double>& event) const {
    EXPECT_EQ(event.get_trigger_type(), TriggerType::kInitialization);
    pub_init_ = true;
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<double>&,
      const std::vector<const DiscreteUpdateEvent<double>*>& events,
      DiscreteValues<double>* discrete_state) const final {
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.at(0)->get_trigger_type(), TriggerType::kInitialization);
    dis_update_init_ = true;
    discrete_state->set_value(Vector1d(1.23));
  }

  void DoCalcUnrestrictedUpdate(
      const Context<double>&,
      const std::vector<const UnrestrictedUpdateEvent<double>*>& events,
      State<double>* state) const final {
    EXPECT_EQ(events.size(), 1);
    EXPECT_EQ(events.at(0)->get_trigger_type(), TriggerType::kInitialization);
    unres_update_init_ = true;
    state->get_mutable_abstract_state<bool>(0) = true;
  }

  mutable bool pub_init_{false};
  mutable bool dis_update_init_{false};
  mutable bool unres_update_init_{false};
};

}  // namespace systems
}  // namespace drake
