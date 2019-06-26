#pragma once

#include <cmath>

namespace fl {
class Twist2d {
 public:
  // Constructors
  Twist2d() : dx_(0.0), dy_(0.0), dtheta_(0.0) {}
  Twist2d(const double dx, const double dy, const double dtheta) : dx_(dx), dy_(dy), dtheta_(dtheta) {}

  // Operator Overloads
  Twist2d operator*(const double scalar) const { return {dx_ * scalar, dy_ * scalar, dtheta_ * scalar}; }

  // Accessors
  double Dx() const { return dx_; }
  double Dy() const { return dy_; }
  double Dtheta() const { return dtheta_; }

  double Norm() const {
    if (dy_ == 0.0) return std::abs(dx_);
    return std::hypot(dx_, dy_);
  }

 private:
  double dx_;
  double dy_;
  double dtheta_;
};
}  // namespace fl
