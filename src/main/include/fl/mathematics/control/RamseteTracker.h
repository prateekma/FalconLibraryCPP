#pragma once

#include "TrajectoryTracker.h"

namespace fl {
class RamseteTracker : public TrajectoryTracker {
 public:
  RamseteTracker(const double beta, const double zeta);

  TrajectoryTrackerVelocityOutput CalculateState(const TimedIterator<Pose2dWithCurvature>& iterator,
                                                 const Pose2d& robot_pose) const override;

 private:
  static double Sinc(const double theta) {
    if (EpsilonEquals(theta, 0.0)) {
      return 1.0 - 1.0 / 6.0 * theta * theta;
    }
    return std::sin(theta) / theta;
  }

  double beta_;
  double zeta_;
};
}  // namespace fl
