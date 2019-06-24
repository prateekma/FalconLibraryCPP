#pragma once

#include <utility>
#include "VaryInterpolatable.h"
#include "Pose2d.h"
#include "Interpolatable.h"

namespace frc5190 {
class Pose2dWithCurvature final
    : public VaryInterpolatable<Pose2dWithCurvature> {
 public:
  // Constructors
  Pose2dWithCurvature(Pose2d pose, const double curvature, const double dkds)
      : pose_(std::move(pose)), curvature_(curvature), dkds_(dkds) {}

  // Overriden Methods
  double Distance(const Pose2dWithCurvature& other) override {
    return pose_.Distance(other.pose_);
  }

  Pose2dWithCurvature Interpolate(const Pose2dWithCurvature& end_value,
                                  double t) override {
    return Pose2dWithCurvature{pose_.Interpolate(end_value.pose_, t),
                               Lerp(curvature_, end_value.curvature_, t),
                               Lerp(dkds_, end_value.dkds_, t)};
  }

  // Operator Overloads
  Pose2dWithCurvature operator+(const Pose2d& other) const {
    return Pose2dWithCurvature{pose_ + other, curvature_, dkds_};
  }

  // Accessors
  const Pose2d& Pose() const { return pose_; }
  double Curvature() const { return curvature_; }
  double Dkds() const { return dkds_; }

  Pose2dWithCurvature Mirror() const {
    return Pose2dWithCurvature{pose_.Mirror(), -curvature_, -dkds_};
  }

 private:
  Pose2d pose_;
  double curvature_;
  double dkds_;
};
}  // namespace frc5190
