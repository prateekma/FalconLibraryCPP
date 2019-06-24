#pragma once

#include <cmath>

#include "Pose2d.h"
#include "Utilities.h"

namespace frc5190 {
class Twist2d {
 public:
  // Constructors
  Twist2d() : dx_(0.0), dy_(0.0), dtheta_(0.0) {}
  Twist2d(const double dx, const double dy, const double dtheta)
      : dx_(dx), dy_(dy), dtheta_(dtheta) {}

  // Operator Overloads
  Twist2d operator*(const double scalar) const {
    return {dx_ * scalar, dy_ * scalar, dtheta_ * scalar};
  }

  // Accessors
  double Dx() const { return dx_; }
  double Dy() const { return dy_; }
  double Dtheta() const { return dtheta_; }

  double Norm() const {
    if (dy_ == 0.0) return std::abs(dx_);
    return std::hypot(dx_, dy_);
  }

  Pose2d AsPose() const {
    const auto sin_theta = std::sin(dtheta_);
    const auto cos_theta = std::cos(dtheta_);

    double s, c;
    if (std::abs(dtheta_) < kEpsilon) {
      s = 1.0 - 1.0 / 6.0 * dtheta_ * dtheta_;
      c = 0.5 * dtheta_;
    } else {
      s = sin_theta / dtheta_;
      c = (1 - cos_theta) / dtheta_;
    }

    return Pose2d{Translation2d{dx_ * s - dy_ * c, dx_ * c + dy_ * s},
                  Rotation2d{cos_theta, sin_theta, false}};
  }

 private:
  double dx_;
  double dy_;
  double dtheta_;
};
}  // namespace frc5190
