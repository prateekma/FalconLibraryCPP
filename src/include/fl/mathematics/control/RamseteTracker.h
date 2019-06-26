#pragma once

#include "TrajectoryTracker.h"

namespace fl {
class RamseteTracker : public TrajectoryTracker {
 public:
  RamseteTracker(const double beta, const double zeta) : beta_(beta), zeta_(zeta) {}

  TrajectoryTrackerVelocityOutput CalculateState(const TimedIterator<Pose2dWithCurvature>& iterator,
                                                 const Pose2d& robot_pose) const override {
    const TimedEntry<Pose2dWithCurvature> reference_state = iterator.CurrentState().state;
    const Pose2d error = reference_state.State().Pose().InFrameOfReferenceOf(robot_pose);

    const auto vd = reference_state.Velocity();
    const auto wd = vd * reference_state.State().Curvature();

    const auto k1          = 2 * zeta_ * std::sqrt(wd * wd + beta_ * vd * vd);
    const auto angle_error = error.Rotation().Radians();

    const auto linear  = vd * error.Rotation().Cos() + k1 * error.Translation().X();
    const auto angular = wd + beta_ * vd * Sinc(angle_error) * error.Translation().Y() + k1 * angle_error;

    return {linear, angular};
  }

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
