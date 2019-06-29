#pragma once

#include "TimingConstraint.h"
#include "fl/mathematics/geometry/Pose2dWithCurvature.h"

namespace fl {

class VelocityLimitRadiusConstraint : public TimingConstraint<Pose2dWithCurvature> {
 public:
  VelocityLimitRadiusConstraint(const Translation2d& point, const double radius, const double max_velocity);

  double                 MaxVelocity(const Pose2dWithCurvature& state) const override;
  fl::MinMaxAcceleration MinMaxAcceleration(const Pose2dWithCurvature& state, double velocity) const override;

 private:
  Translation2d point_;
  double        radius_;
  double        max_velocity_;
};

}  // namespace fl
