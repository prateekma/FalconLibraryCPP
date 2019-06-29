#include "fl/mathematics/trajectory/constraints/VelocityLimitRadiusConstraint.h"

namespace fl {

VelocityLimitRadiusConstraint::VelocityLimitRadiusConstraint(const Translation2d& point, const double radius,
                                                             const double max_velocity)
    : point_(point), radius_(radius), max_velocity_(max_velocity) {}

double VelocityLimitRadiusConstraint::MaxVelocity(const Pose2dWithCurvature& state) const {
  if (state.Pose().Translation().Distance(point_) < radius_) {
    return max_velocity_;
  }
  return std::numeric_limits<double>::max();
}

fl::MinMaxAcceleration VelocityLimitRadiusConstraint::MinMaxAcceleration(const Pose2dWithCurvature& state,
                                                                         double velocity) const {
  return kNoLimits;
}
}  // namespace fl
