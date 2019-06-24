#pragma once
#include "../geometry/Pose2dWithCurvature.h"

namespace frc5190 {
class ParametricSpline {
 public:
  virtual Translation2d Point(double t) const = 0;
  virtual Rotation2d Heading(double t) const = 0;
  virtual double Curvature(double t) const = 0;
  virtual double DCurvature(double t) const = 0;
  virtual double Velocity(double t) const = 0;

  Pose2dWithCurvature PoseWithCurvature(const double t) const {
    return Pose2dWithCurvature{Pose(t), Curvature(t),
                               DCurvature(t) / Velocity(t)};
  }

 private:
  Pose2d Pose(const double t) const { return Pose2d{Point(t), Heading(t)}; }
};
}  // namespace frc5190
