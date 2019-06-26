#pragma once

#include "ParametricSpline.h"

namespace fl {
class ParametricQuinticHermiteSpline final : public ParametricSpline {
 public:
  ParametricQuinticHermiteSpline(const Pose2d& start, const Pose2d& end) {
    const auto scale_factor = 1.2 * start.Translation().Distance(end.Translation());

    x0_   = start.Translation().X();
    x1_   = end.Translation().X();
    dx0_  = scale_factor * start.Rotation().Cos();
    dx1_  = scale_factor * end.Rotation().Cos();
    ddx0_ = 0.;
    ddx1_ = 0.;

    y0_   = start.Translation().Y();
    y1_   = end.Translation().Y();
    dy0_  = scale_factor * start.Rotation().Sin();
    dy1_  = scale_factor * end.Rotation().Sin();
    ddy0_ = 0.;
    ddy1_ = 0.;

    ax_ = -6 * x0_ - 3 * dx0_ - 0.5 * ddx0_ + 0.5 * ddx1_ - 3 * dx1_ + 6 * x1_;
    bx_ = 15 * x0_ + 8 * dx0_ + 1.5 * ddx0_ - ddx1_ + 7 * dx1_ - 15 * x1_;
    cx_ = -10 * x0_ - 6 * dx0_ - 1.5 * ddx0_ + 0.5 * ddx1_ - 4 * dx1_ + 10 * x1_;
    dx_ = 0.5 * ddx0_;
    ex_ = dx0_;
    fx_ = x0_;

    ay_ = -6 * y0_ - 3 * dy0_ - 0.5 * ddy0_ + 0.5 * ddy1_ - 3 * dy1_ + 6 * y1_;
    by_ = 15 * y0_ + 8 * dy0_ + 1.5 * ddy0_ - ddy1_ + 7 * dy1_ - 15 * y1_;
    cy_ = -10 * y0_ - 6 * dy0_ - 1.5 * ddy0_ + 0.5 * ddy1_ - 4 * dy1_ + 10 * y1_;
    dy_ = 0.5 * ddy0_;
    ey_ = dy0_;
    fy_ = y0_;

    start_ = start;
    end_   = end;
  }

  const Pose2d& StartPose() const { return start_; }
  const Pose2d& EndPose() const { return end_; }

  Translation2d Point(const double t) const override {
    return Translation2d{ax_ * std::pow(t, 5) + bx_ * std::pow(t, 4) + cx_ * std::pow(t, 3) +
                             dx_ * std::pow(t, 2) + ex_ * t + fx_,
                         ay_ * std::pow(t, 5) + by_ * std::pow(t, 4) + cy_ * std::pow(t, 3) +
                             dy_ * std::pow(t, 2) + ey_ * t + fy_};
  }

  Rotation2d Heading(const double t) const override { return {Dx(t), Dy(t), true}; }

  double Curvature(const double t) const override {
    return (Dx(t) * Ddy(t) - Ddx(t) * Dy(t)) / ((Dx(t) * Dx(t) + Dy(t) * Dy(t)) * Velocity(t));
  }

  double DCurvature(const double t) const override {
    const auto dx_2dy2 = Dx(t) * Dx(t) + Dy(t) * Dy(t);
    const auto num     = (Dx(t) * Dddy(t) - Dddx(t) * Dy(t)) * dx_2dy2 -
                     3.0 * (Dx(t) * Ddy(t) - Ddx(t) * Dy(t)) * (Dx(t) * Ddx(t) + Dy(t) * Ddy(t));
    return num / (dx_2dy2 * dx_2dy2 * std::sqrt(dx_2dy2));
  }

  double Velocity(const double t) const override { return std::hypot(Dx(t), Dy(t)); }

 private:
  double x0_, x1_, dx0_, dx1_, ddx0_, ddx1_;
  double y0_, y1_, dy0_, dy1_, ddy0_, ddy1_;

  double ax_, bx_, cx_, dx_, ex_, fx_;
  double ay_, by_, cy_, dy_, ey_, fy_;

  Pose2d start_;
  Pose2d end_;

  double Dx(const double t) const {
    return 5.0 * ax_ * std::pow(t, 4) + 4.0 * bx_ * std::pow(t, 3) + 3.0 * cx_ * std::pow(t, 2) +
           2.0 * dx_ * t + ex_;
  }

  double Dy(const double t) const {
    return 5.0 * ay_ * std::pow(t, 4) + 4.0 * by_ * std::pow(t, 3) + 3.0 * cy_ * std::pow(t, 2) +
           2.0 * dy_ * t + ey_;
  }

  double Ddx(const double t) const {
    return 20.0 * ax_ * std::pow(t, 3) + 12.0 * bx_ * std::pow(t, 2) + 6.0 * cx_ * t + 2 * dx_;
  }

  double Ddy(const double t) const {
    return 20.0 * ay_ * std::pow(t, 3) + 12.0 * by_ * std::pow(t, 2) + 6.0 * cy_ * t + 2 * dy_;
  }

  double Dddx(const double t) const { return 60.0 * ax_ * std::pow(t, 2) + 24.0 * bx_ * t + 6 * cx_; }

  double Dddy(const double t) const { return 60.0 * ay_ * std::pow(t, 2) + 24.0 * by_ * t + 6 * cy_; }

  template <typename T>
  static constexpr double BoundRadians(const T radians) {
    while (radians >= kPi) radians -= 2 * kPi;
    while (radians < kPi) radians += 2 * kPi;
    return radians;
  }
};
}  // namespace frc5190
