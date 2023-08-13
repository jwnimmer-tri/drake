// clang-format off
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/event_collection.h"
// clang-format on

// Due to a circular dependency (the event types depend on the collection types
// and vice versa) it's not possible to implement the event classes in separate
// translation units. This file contains the implementations for both event.h
// and event_collection.h.

namespace drake {
namespace systems {

template <typename T>
Event<T>::~Event() = default;

namespace internal {

template <typename T, typename Derived, typename Output>
bool EventImpl<T, Derived, Output>::is_discrete_update() const {
  return std::is_same_v<Derived, DiscreteUpdateEvent<T>>;
}

template <typename T, typename Derived, typename Output>
std::unique_ptr<Event<T>> EventImpl<T, Derived, Output>::Clone() const {
  return std::make_unique<Derived>(derived());
}

template <typename T, typename Derived, typename Output>
EventImpl<T, Derived, Output>::~EventImpl() = default;

template <typename T, typename Derived, typename Output>
void EventImpl<T, Derived, Output>::DoAddToComposite(
    TriggerType trigger_type, CompositeEventCollection<T>* events) const {
  Derived event(derived());
  event.set_trigger_type(trigger_type);
  events->AddEvent(std::move(event));
}

}  // namespace internal

template <typename T>
PublishEvent<T>::~PublishEvent() = default;

template <typename T>
 EventStatus PublishEvent<T>::InvokeCallback(
    const System<T>& system, const Context<T>& context) const {
  const auto& callback = this->get_callback();
  if (callback == nullptr) {
    return EventStatus::DidNothing();
  }
  return callback(system, context, *this);
}

template <typename T>
DiscreteUpdateEvent<T>::~DiscreteUpdateEvent() = default;

template <typename T>
EventStatus DiscreteUpdateEvent<T>::InvokeCallback(
    const System<T>& system, const Context<T>& context,
    DiscreteValues<T>* discrete_state) const {
  const auto& callback = this->get_callback();
  if (callback == nullptr) {
    return EventStatus::DidNothing();
  }
  return callback(system, context, *this, discrete_state);
}

template <typename T>
UnrestrictedUpdateEvent<T>::~UnrestrictedUpdateEvent() = default;

template <typename T>
EventStatus UnrestrictedUpdateEvent<T>::InvokeCallback(
    const System<T>& system, const Context<T>& context, State<T>* state) const {
  const auto& callback = this->get_callback();
  if (callback == nullptr) {
    return EventStatus::DidNothing();
  }
  return callback(system, context, *this, state);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename T>
PublishEvent<T>::PublishEvent(const PublishCallback& callback) {
  set_callback([callback](const System<T>&, const Context<T>& context,
                          const PublishEvent<T>& event) {
    callback(context, event);
    return EventStatus::Succeeded();
  });
}
template <typename T>
PublishEvent<T>::PublishEvent(const SystemCallback& callback) {
  set_callback([callback](const System<T>& system, const Context<T>& context,
                          const PublishEvent<T>& event) {
    callback(system, context, event);
    return EventStatus::Succeeded();
  });
}
template <typename T>
DiscreteUpdateEvent<T>::DiscreteUpdateEvent(
    const DiscreteUpdateCallback& callback) {
  set_callback([callback](const System<T>&, const Context<T>& context,
                          const DiscreteUpdateEvent<T>& event,
                          DiscreteValues<T>* discrete_state) {
    callback(context, event, discrete_state);
    return EventStatus::Succeeded();
  });
}
template <typename T>
DiscreteUpdateEvent<T>::DiscreteUpdateEvent(const SystemCallback& callback) {
  set_callback([callback](const System<T>& system, const Context<T>& context,
                          const DiscreteUpdateEvent<T>& event,
                          DiscreteValues<T>* discrete_state) {
    callback(system, context, event, discrete_state);
    return EventStatus::Succeeded();
  });
}
template <typename T>
UnrestrictedUpdateEvent<T>::UnrestrictedUpdateEvent(
    const UnrestrictedUpdateCallback& callback) {
  set_callback([callback](const System<T>&, const Context<T>& context,
                          const UnrestrictedUpdateEvent<T>& event,
                          State<T>* state) {
    callback(context, event, state);
    return EventStatus::Succeeded();
  });
}
template <typename T>
UnrestrictedUpdateEvent<T>::UnrestrictedUpdateEvent(
    const SystemCallback& callback) {
  set_callback([callback](const System<T>& system, const Context<T>& context,
                          const UnrestrictedUpdateEvent<T>& event,
                          State<T>* state) {
    callback(system, context, event, state);
    return EventStatus::Succeeded();
  });
}
#pragma GCC diagnostic pop

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::CompositeEventCollection)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafCompositeEventCollection)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramCompositeEventCollection)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::WitnessTriggeredEventData)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Event)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PublishEvent)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteUpdateEvent)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::UnrestrictedUpdateEvent)
