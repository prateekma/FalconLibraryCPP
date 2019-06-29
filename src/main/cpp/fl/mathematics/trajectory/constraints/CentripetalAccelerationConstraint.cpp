#include "fl/mathematics/trajectory/constraints/CentripetalAccelerationConstraint.h"

namespace fl {

CentripetalAccelerationConstraint::CentripetalAccelerationConstraint(
    const double max_centripetal_acceleration)
    : max_centripetal_acceleration_(max_centripetal_acceleration) {}

double CentripetalAccelerationConstraint::MaxVelocity(const Pose2dWithCurvature& state) const {
  return std::sqrt(std::abs(max_centripetal_acceleration_ / state.Curvature()));
}

fl::MinMaxAcceleration CentripetalAccelerationConstraint::MinMaxAcceleration(const Pose2dWithCurvature& state,
                                                                             double velocity) const {
  return kNoLimits;
}
}  // namespace fl
