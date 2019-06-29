#pragma once

#include "TrajectoryTracker.h"
#include "fl/mathematics/geometry/Pose2d.h"
#include "fl/mathematics/geometry/Pose2dWithCurvature.h"

namespace fl {
class PurePursuitTracker : public TrajectoryTracker {
 public:
  PurePursuitTracker(const double lat, const units::second_t lookahead_time,
                     const double min_lookahead_distance);

  TrajectoryTrackerVelocityOutput CalculateState(const TimedIterator<Pose2dWithCurvature>& iterator,
                                                 const Pose2d& robot_pose) const override;

 private:
  double          lat_;
  units::second_t lookahead_time_;
  double          min_lookahead_distance_;

  Pose2d CalculateLookaheadPose(const TimedIterator<Pose2dWithCurvature>& iterator,
                                const Pose2d&                             robot_pose) const;
};
}  // namespace fl