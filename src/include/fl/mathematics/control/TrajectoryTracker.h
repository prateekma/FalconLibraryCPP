#pragma once

#include "fl/mathematics/trajectory/TimedTrajectory.h"

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
    previous_time_     = -1.;
  }

  TrajectoryTrackerOutput NextState(const Pose2d& current_pose, const double current_time) {
    if (iterator_ == nullptr) throw std::exception("Iterator was nullptr.");
    auto& iterator = *iterator_;

    const auto dt  = (previous_time_ < 0.0) ? 0.0 : current_time - previous_time_;
    previous_time_ = current_time;

    iterator.Advance(dt);

    const auto velocity = CalculateState(iterator, current_pose);

    const auto linear_velocity  = velocity.linear_velocity;
    const auto angular_velocity = velocity.angular_velocity;

    if (previous_velocity_ == nullptr || dt <= 0.) {
      previous_velocity_.reset(new TrajectoryTrackerVelocityOutput{linear_velocity, angular_velocity});
      return {linear_velocity, 0.0, angular_velocity, 0.0};
    }

    const auto previous_velocity = *previous_velocity_;

    previous_velocity_->linear_velocity  = linear_velocity;
    previous_velocity_->angular_velocity = angular_velocity;

    return {linear_velocity, (linear_velocity - previous_velocity.linear_velocity) / dt, angular_velocity,
            (angular_velocity - previous_velocity.angular_velocity) / dt};
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
  double                                           previous_time_ = -1.;
};
}  // namespace fl
