#pragma once

#include "fl/mathematics/trajectory/TimedTrajectory.h"

#include <units.h>
#include <memory>

namespace fl {

struct TrajectoryTrackerVelocityOutput {
  double linear_velocity  = 0.0;
  double angular_velocity = 0.0;
};

struct TrajectoryTrackerOutput {
  double linear_velocity      = 0.0;
  double linear_acceleration  = 0.0;
  double angular_velocity     = 0.0;
  double angular_acceleration = 0.0;
};

class TrajectoryTracker {
 public:
  void Reset(const TimedTrajectory<Pose2dWithCurvature>& trajectory) {
    iterator_          = static_cast<TimedIterator<Pose2dWithCurvature>*>(trajectory.Iterator().get());
    previous_velocity_ = nullptr;
    previous_time_     = units::second_t(-1);
  }

  TrajectoryTrackerOutput NextState(const Pose2d& current_pose, const units::second_t current_time) {
    if (iterator_ == nullptr) throw std::exception("Iterator was nullptr.");
    auto& iterator = *iterator_;

    const auto dt = (units::unit_cast<double>(previous_time_) < 0.0) ? units::second_t{0.0}
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
        (angular_velocity - previous_velocity_->angular_velocity) / _dt};

    previous_velocity_->linear_velocity  = linear_velocity;
    previous_velocity_->angular_velocity = angular_velocity;

    return output;
  }

  virtual TrajectoryTrackerVelocityOutput CalculateState(const TimedIterator<Pose2dWithCurvature>& iterator,
                                                         const Pose2d& robot_pose) const = 0;

  TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> ReferencePoint() const {
    return iterator_->CurrentState();
  }

  bool IsFinished() const { return iterator_->IsDone(); }

 private:
  TimedIterator<Pose2dWithCurvature>*              iterator_ = nullptr;
  std::unique_ptr<TrajectoryTrackerVelocityOutput> previous_velocity_;
  units::second_t                                  previous_time_ = units::second_t(-1);
};
}  // namespace fl
