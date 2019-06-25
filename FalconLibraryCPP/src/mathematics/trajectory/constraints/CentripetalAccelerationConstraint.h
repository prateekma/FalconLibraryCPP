#pragma once

#include "../../geometry/Pose2dWithCurvature.h"
#include "TimingConstraint.h"

namespace frc5190 {

class CentripetalAccelerationConstraint final
    : public TimingConstraint<Pose2dWithCurvature> {

 public:
  explicit CentripetalAccelerationConstraint(
      const double max_centripetal_acceleration)
      : max_centripetal_acceleration_(max_centripetal_acceleration) {}

  ~CentripetalAccelerationConstraint() = default;

  double MaxVelocity(const Pose2dWithCurvature& state) const override {
    return std::sqrt(
        std::abs(max_centripetal_acceleration_ / state.Curvature()));
  }

  frc5190::MinMaxAcceleration MinMaxAcceleration(
      const Pose2dWithCurvature& state, double velocity) const override {
    return kNoLimits;
  }

 private:
  double max_centripetal_acceleration_;
};

}  // namespace frc5190
