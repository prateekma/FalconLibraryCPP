#pragma once

#include "TimingConstraint.h"
#include "fl/mathematics/geometry/Pose2dWithCurvature.h"

namespace fl {

class VelocityLimitRadiusConstraint : public TimingConstraint<Pose2dWithCurvature> {
 public:
  VelocityLimitRadiusConstraint(const Translation2d& point, const double radius, const double max_velocity)
      : point_(point), radius_(radius), max_velocity_(max_velocity) {}

  ~VelocityLimitRadiusConstraint() = default;

  double MaxVelocity(const Pose2dWithCurvature& state) const override {
    if (state.Pose().Translation().Distance(point_) < radius_) {
      return max_velocity_;
    }
    return std::numeric_limits<double>::max();
  }

  fl::MinMaxAcceleration MinMaxAcceleration(const Pose2dWithCurvature& state,
                                            double                     velocity) const override {
    return kNoLimits;
  }

 private:
  Translation2d point_;
  double        radius_;
  double        max_velocity_;
};

}  // namespace fl
