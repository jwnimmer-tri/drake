#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {
namespace trajectories {

/** A WrappedTrajectory delegates all calls to a nested Trajectory object
maintained as a shared_ptr. Copying this object is a *shallow* copy.
@tparam_default_scalar */
template <typename T>
class WrappedTrajectory final : public Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(WrappedTrajectory);

  /** Wraps the given trajectory, which must not be nullptr. */
  explicit WrappedTrajectory(std::shared_ptr<const Trajectory<T>> trajectory);

  ~WrappedTrajectory() final;

  // Overrides for the Trajectory base class.
  std::unique_ptr<Trajectory<T>> Clone() const final;
  MatrixX<T> value(const T& t) const final;
  Eigen::Index rows() const final;
  Eigen::Index cols() const final;
  T start_time() const final;
  T end_time() const final;

  /** (Internal use only) Returns the underlying Trajectory. */
  const Trajectory<T>* unwrap() const;

 private:
  void ThrowIfEmpty(const char* func) const;

  // Overrides for the Trajectory base class.
  bool do_has_derivative() const final;
  MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const final;
  std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const final;

  std::shared_ptr<const Trajectory<T>> trajectory_;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::WrappedTrajectory);
