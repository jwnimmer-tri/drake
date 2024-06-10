#include "drake/systems/analysis/realtime_rate_calculator.h"

#include <utility>

namespace drake {
namespace systems {
namespace internal {

static const double report_period = 0.25;

std::optional<double>
RealtimeRateCalculator::UpdateAndRecalculate(double current_sim_time) {
  // First-time initialization.
  if (!initialized_) {
    initialized_ = true;
    timer_->Start();
    next_report_ = 1;
    prev_sim_time_ = current_sim_time;
    prev_wall_time_ = 0;
    return std::nullopt;
  }

  // Only report a result when the wall clock has sufficiently advanced.
  const double current_wall_time = timer_->Tick();
  if (!(current_wall_time >= next_report_ * report_period)) {
    return std::nullopt;
  }

  // Only report positive progress (not zero nor negative).
  const double sim_delta{current_sim_time - prev_sim_time_};
  if (!(sim_delta > 0)) {
    return std::nullopt;
  }

  // Only report positive progress (not zero).
  const double wall_delta{current_wall_time - prev_wall_time_};
  if (!(wall_delta > 0)) {
    return std::nullopt;
  }

  // We have a valid measurement.
  const double result = sim_delta / wall_delta;
  ++next_report_;
  prev_sim_time_ = current_sim_time;
  prev_wall_time_ = current_wall_time;

  return result;
}

void RealtimeRateCalculator::InjectMockTimer(
    std::unique_ptr<Timer> mock_timer) {
  timer_ = std::move(mock_timer);
}

}  // namespace internal
}  // namespace systems
}  // namespace drake
