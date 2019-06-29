#pragma once

#include "TimingConstraint.h"
#include "fl/mathematics/geometry/Pose2dWithCurvature.h"

namespace fl {

class AngularAccelerationConstraint final : public TimingConstraint<Pose2dWithCurvature> {
 public:
  explicit AngularAccelerationConstraint(double max_angular_acceleration);

  double                 MaxVelocity(const Pose2dWithCurvature& state) const override;
  fl::MinMaxAcceleration MinMaxAcceleration(const Pose2dWithCurvature& state, double velocity) const override;

 private:
  double max_angular_acceleration_;
};

}  // namespace fl
