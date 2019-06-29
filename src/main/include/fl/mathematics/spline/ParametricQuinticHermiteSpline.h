#pragma once

#include "ParametricSpline.h"

namespace fl {
class ParametricQuinticHermiteSpline final : public ParametricSpline {
 public:
  ParametricQuinticHermiteSpline(const Pose2d& start, const Pose2d& end);

  const Pose2d& StartPose() const;
  const Pose2d& EndPose() const;

  Translation2d Point(const double t) const override;
  Rotation2d    Heading(const double t) const override;
  double        Curvature(const double t) const override;
  double        DCurvature(const double t) const override;
  double        Velocity(const double t) const override;

 private:
  double x0_, x1_, dx0_, dx1_, ddx0_, ddx1_;
  double y0_, y1_, dy0_, dy1_, ddy0_, ddy1_;

  double ax_, bx_, cx_, dx_, ex_, fx_;
  double ay_, by_, cy_, dy_, ey_, fy_;

  Pose2d start_;
  Pose2d end_;

  double Dx(const double t) const;
  double Dy(const double t) const;
  double Ddx(const double t) const;
  double Ddy(const double t) const;
  double Dddx(const double t) const;
  double Dddy(const double t) const;
};
}  // namespace fl
