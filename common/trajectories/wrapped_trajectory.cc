#include "drake/common/trajectories/wrapped_trajectory.h"

namespace drake {
namespace trajectories {

template <typename T>
WrappedTrajectory<T>::WrappedTrajectory(
    std::shared_ptr<const Trajectory<T>> trajectory)
    : trajectory_(std::move(trajectory)) {
  DRAKE_THROW_UNLESS(trajectory_ != nullptr);
}

template <typename T>
WrappedTrajectory<T>::~WrappedTrajectory() = default;

template <typename T>
std::unique_ptr<Trajectory<T>> WrappedTrajectory<T>::Clone() const {
  ThrowIfEmpty(__func__);
  return std::make_unique<WrappedTrajectory<T>>(trajectory_);
}

template <typename T>
MatrixX<T> WrappedTrajectory<T>::value(const T& t) const {
  ThrowIfEmpty(__func__);
  return trajectory_->value(t);
}

template <typename T>
Eigen::Index WrappedTrajectory<T>::rows() const {
  ThrowIfEmpty(__func__);
  return trajectory_->rows();
}

template <typename T>
Eigen::Index WrappedTrajectory<T>::cols() const {
  ThrowIfEmpty(__func__);
  return trajectory_->cols();
}

template <typename T>
T WrappedTrajectory<T>::start_time() const {
  ThrowIfEmpty(__func__);
  return trajectory_->start_time();
}

template <typename T>
T WrappedTrajectory<T>::end_time() const {
  ThrowIfEmpty(__func__);
  return trajectory_->end_time();
}

template <typename T>
const Trajectory<T>* WrappedTrajectory<T>::unwrap() const {
  return trajectory_.get();
}

template <typename T>
void WrappedTrajectory<T>::ThrowIfEmpty(const char* func) const {
  if (trajectory_ == nullptr) {
    throw std::logic_error(fmt::format(
        "Invalid call to WrappedTrajectory::{}() on a moved-from object",
        func));
  }
}

template <typename T>
bool WrappedTrajectory<T>::do_has_derivative() const {
  ThrowIfEmpty(__func__);
  return trajectory_->has_derivative();
}

template <typename T>
MatrixX<T> WrappedTrajectory<T>::DoEvalDerivative(const T& t,
                                                  int derivative_order) const {
  ThrowIfEmpty(__func__);
  return trajectory_->EvalDerivative(t, derivative_order);
}

template <typename T>
std::unique_ptr<Trajectory<T>> WrappedTrajectory<T>::DoMakeDerivative(
    int derivative_order) const {
  ThrowIfEmpty(__func__);
  return trajectory_->MakeDerivative(derivative_order);
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::WrappedTrajectory);
