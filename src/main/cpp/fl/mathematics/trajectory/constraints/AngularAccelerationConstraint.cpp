#include "fl/mathematics/trajectory/constraints/AngularAccelerationConstraint.h"

namespace fl {

AngularAccelerationConstraint::AngularAccelerationConstraint(double max_angular_acceleration)
    : max_angular_acceleration_(max_angular_acceleration) {}

double AngularAccelerationConstraint::MaxVelocity(const Pose2dWithCurvature& state) const {
  /**
   * We don't want v^2 * dk/ds alone to go over the max angular acceleration.
   * v^2 * dk/ds = maxAngularAcceleration when linear acceleration = 0.
   * v = sqrt(maxAngularAcceleration / dk/ds)
   */

  return std::sqrt(max_angular_acceleration_ / std::abs(state.Dkds()));
}

fl::MinMaxAcceleration AngularAccelerationConstraint::MinMaxAcceleration(const Pose2dWithCurvature& state,
                                                                         const double velocity) const {
  /**
   * We want to limit the acceleration such that we never go above the
   *
   * Angular acceleration = dw/dt     WHERE   w = omega = angular velocity
   * w = v * k                        WHERE   v = linear velocity, k =
   * curvature
   *
   * dw/dt = d/dt (v * k)
   *
   * By chain rule,
   * dw/dt = dv/dt * k + v * dk/dt   [1]
   *
   * We don't have dk/dt, but we do have dk/ds and ds/dt
   * dk/ds * ds/dt = dk/dt     [2]
   *
   * Substituting [2] in [1], we get
   * dw/dt = acceleration * curvature + velocity * velocity * d_curvature
   * WHERE acceleration = dv/dt, velocity = ds/dt, d_curvature = dk/dt and
   * curvature = k
   *
   * We now want to find the linear acceleration such that the angular
   * acceleration (dw/dt) never goes above the specified amount.
   *
   * acceleration * curvature = dw/dt - (velocity * velocity * d_curvature)
   * acceleration = (dw/dt - (velocity * velocity * d_curvature)) / curvature
   */

  const auto max_absolute_acceleration =
      std::abs((max_angular_acceleration_ - (velocity * velocity * state.Dkds())) / state.Curvature());

  return fl::MinMaxAcceleration{-max_absolute_acceleration, max_absolute_acceleration};
}
}  // namespace fl
