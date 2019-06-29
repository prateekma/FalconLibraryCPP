#include "fl/mathematics/control/TrajectoryTracker.h"

namespace fl {

void TrajectoryTracker::Reset(const TimedTrajectory<Pose2dWithCurvature>& trajectory) {
  iterator_          = static_cast<TimedIterator<Pose2dWithCurvature>*>(trajectory.Iterator().get());
  previous_velocity_ = nullptr;
  previous_time_     = units::second_t(-1);
}

TrajectoryTrackerOutput TrajectoryTracker::NextState(const Pose2d&         current_pose,
                                                     const units::second_t current_time) {
  if (iterator_ == nullptr) throw std::exception("Iterator was nullptr.");
  auto& iterator = *iterator_;

  const auto dt = (units::unit_cast<double>(previous_time_) < 0.0)
                    ? units::second_t{0.0}
                    : current_time - previous_time_;
  previous_time_ = current_time;

  iterator.Advance(dt);

  const auto velocity = CalculateState(iterator, current_pose);

  const auto linear_velocity  = velocity.linear_velocity;
  const auto angular_velocity = velocity.angular_velocity;

  if (previous_velocity_ == nullptr || dt <= units::second_t()) {
    previous_velocity_.reset(new TrajectoryTrackerVelocityOutput{linear_velocity, angular_velocity});
    return {linear_velocity, 0.0, angular_velocity, 0.0};
  }

  const auto _dt = units::unit_cast<double>(dt);

  const TrajectoryTrackerOutput output{
    linear_velocity, (linear_velocity - previous_velocity_->linear_velocity) / _dt, angular_velocity,
    (angular_velocity - previous_velocity_->angular_velocity) / _dt
  };

  previous_velocity_->linear_velocity  = linear_velocity;
  previous_velocity_->angular_velocity = angular_velocity;

  return output;
}

TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> TrajectoryTracker::ReferencePoint() const {
  return iterator_->CurrentState();
}

bool TrajectoryTracker::IsFinished() const { return iterator_->IsDone(); }
}
