#pragma once

#include <utility>
#include "Pose2d.h"

namespace fl {
class Pose2dWithCurvature final : public VaryInterpolatable<Pose2dWithCurvature> {
 public:
  // Constructors
  Pose2dWithCurvature();
  Pose2dWithCurvature(Pose2d pose, const double curvature, const double dkds);

  // Overriden Methods
  double Distance(const Pose2dWithCurvature& other) const override;

  Pose2dWithCurvature Interpolate(const Pose2dWithCurvature& end_value, double t) const override;

  // Operator Overloads
  Pose2dWithCurvature operator+(const Pose2d& other) const;

  // Accessors
  const Pose2d& Pose() const;
  double        Curvature() const;
  double        Dkds() const;

  Pose2dWithCurvature Mirror() const;

 private:
  Pose2d pose_;
  double curvature_;
  double dkds_;
};
}  // namespace fl
