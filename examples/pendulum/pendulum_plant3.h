#pragma once

#include <cmath>
#include <functional>
#include <tuple>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"

namespace drake {
namespace examples {
namespace pendulum {

enum class AttrKind {
  kInput,
  kOutput,
  kState,
};

// TODO: This should be a GUID (open), not an enum (closed).
enum class WellKnownOutput {
  kPotentialEnergy,
  kKineticEnergy,
};

// Term<ValueType>
// \ Atom
// \ Func(Term...)
//
// A Term has a name.
// A Term can pub/sub invalidation events.
// A Term is assumed untracked by default (never-valid).
//
// An Atom may express an updater.
// - Atoms with no updater are "parameters".
// - Atoms with updates are "state".
//
// An Atom can be bonded to a Term, to change it to an "input".
// - Should we allow "requires bonding" or assume default-fixed inputs?

template <typename T>
class Attr {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Attr)
  Attr(std::string name, const T& value) : value_(value) {}
  const T& operator ()() const { return value_; }
 private:
  T value_{};
};

template <typename T>
class System3 {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(System3)
  System3() = default;
  void DeclareAttr(std::initializer_list<Attr<T>*>) {}
  void DeclareContinuousUpdate(Attr<T>*, Attr<T>*);
  template <typename Class, typename Return, typename... Args>
  void DeclareContinuousUpdate(Attr<T>*, Return (Class::*calc)(Args...) const) {}
  template <typename Class, typename Return, typename... Args>
  void DeclareOutput(WellKnownOutput, Return (Class::*calc)(Args...) const) {}
 private:
};

/** A model of a simple pendulum
@f[ ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = \tau @f]
*/
template <typename T>
class PendulumPlant3 final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PendulumPlant3)

  /** Constructs a default plant. */
  PendulumPlant3() {
    system_.DeclareAttr({&mass_, &length_, &damping_, &gravity_, &tau_,
        &theta_, &thetadot_});
    system_.DeclareContinuousUpdate(&theta_, &thetadot_);
    system_.DeclareContinuousUpdate(
        &thetadot_, &PendulumPlant3::thetadotdot);
    system_.DeclareOutput(
        WellKnownOutput::kPotentialEnergy, &PendulumPlant3::potential_energy);
    system_.DeclareOutput(
        WellKnownOutput::kKineticEnergy, &PendulumPlant3::kinetic_energy);
    // TODO: system_.DeclareScalarConversion(...)
  }

  /** The simple pendulum has a point mass at the end of the arm, in kg. */
  const Attr<T>& mass() const { return mass_; }
  Attr<T>& mass() { return mass_; }

  /** The length of the pendulum arm, in m. */
  const Attr<T>& length() const { return length_; }
  Attr<T>& length() { return length_; }

  /** The damping friction coefficient relating angular velocity to torque,
  in kg m^2/s. */
  const Attr<T>& damping() const { return damping_; }
  Attr<T>& damping() { return damping_; }

  /** An approximate value for gravitational acceleration, in m/s^2. */
  const Attr<T>& gravity() const { return gravity_; }
  Attr<T>& gravity() { return gravity_; }

  /** Torque at the joint, in Newton-meters. */
  const Attr<T>& tau() const { return tau_; }
  Attr<T>& tau() { return tau_; }

  /** The angle of the pendulum, in radians. */
  const Attr<T>& theta() const { return theta_; }
  Attr<T>& theta() { return theta_; }

  /** The angular velocity of the pendulum, in radians/sec. */
  const Attr<T>& thetadot() const { return thetadot_; }
  Attr<T>& thetadot() { return thetadot_; }

  /** The angular acceleration of the pendulum, in radians/sec^2. */
  T thetadotdot() const {
    // TODO: This should probably be formulated as a cache entry (aka, a
    // derived attr).
    const T& m = mass_();
    const T& g = gravity_();
    const T& l = length_();
    const T& d = damping_();
    return
        (tau_() - m * g * l * sin(theta_()) - d * thetadot_()) /
        (m * l * l);
  }

  /** The potential energy = -mgl cos θ. */
  T potential_energy() const {
    using std::cos;
    return -mass_() * gravity_() * length_() * cos(theta_());
  }

  /** The kinetic energy = 1/2 m l² θ̇ ². */
  T kinetic_energy() const {
    using std::pow;
    return 0.5 * mass_() * pow(length_() * thetadot_(), 2);
  }

  /** The kinetic + potential energy. */
  T total_energy() const {
    return potential_energy() + kinetic_energy();
  }

 private:
  Attr<T> mass_{"mass", 1.0};
  Attr<T> length_{"length", 0.5};
  Attr<T> damping_{"damping", 0.1};
  Attr<T> gravity_{"gravity", 9.81};
  Attr<T> tau_{"tau", 0.0};
  Attr<T> theta_{"theta", 0.0};
  Attr<T> thetadot_{"thetadot", 0.0};
  System3<T> system_;
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
