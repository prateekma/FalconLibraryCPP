#include "fl/mathematics/control/RamseteTracker.h"

namespace fl {

RamseteTracker::RamseteTracker(const double beta, const double zeta) : beta_(beta), zeta_(zeta) {}

TrajectoryTrackerVelocityOutput RamseteTracker::CalculateState(
    const TimedIterator<Pose2dWithCurvature>& iterator, const Pose2d& robot_pose) const {
  const TimedEntry<Pose2dWithCurvature> reference_state = iterator.CurrentState().state;
  const Pose2d error = reference_state.State().Pose().InFrameOfReferenceOf(robot_pose);

  const auto vd = units::unit_cast<double>(reference_state.Velocity());
  const auto wd = vd * reference_state.State().Curvature();

  const auto k1          = 2 * zeta_ * std::sqrt(wd * wd + beta_ * vd * vd);
  const auto angle_error = error.Rotation().Radians();

  const auto linear  = vd * error.Rotation().Cos() + k1 * error.Translation().X();
  const auto angular = wd + beta_ * vd * Sinc(angle_error) * error.Translation().Y() + k1 * angle_error;

  return {linear, angular};
}
}  // namespace fl
