#pragma once

#include "fl/mathematics/trajectory/TimedTrajectory.h"

#include <units.h>
#include <memory>
#include "fl/mathematics/geometry/Pose2dWithCurvature.h"

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
  void                    Reset(const TimedTrajectory<Pose2dWithCurvature>& trajectory);
  TrajectoryTrackerOutput NextState(const Pose2d& current_pose, const units::second_t current_time);
  TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> ReferencePoint() const;
  bool                                                   IsFinished() const;

  virtual TrajectoryTrackerVelocityOutput CalculateState(const TimedIterator<Pose2dWithCurvature>& iterator,
                                                         const Pose2d& robot_pose) const = 0;

 private:
  TimedIterator<Pose2dWithCurvature>*              iterator_ = nullptr;
  std::unique_ptr<TrajectoryTrackerVelocityOutput> previous_velocity_;
  units::second_t                                  previous_time_ = units::second_t(-1);
};
}  // namespace fl
