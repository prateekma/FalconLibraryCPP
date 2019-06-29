#include "fl/mathematics/geometry/Twist2d.h"

namespace fl {

Twist2d::Twist2d()
  : dx_(0.0),
    dy_(0.0),
    dtheta_(0.0) {
}

Twist2d::Twist2d(const double dx, const double dy, const double dtheta)
  : dx_(dx),
    dy_(dy),
    dtheta_(dtheta) {
}

Twist2d Twist2d::operator*(const double scalar) const {
  return {dx_ * scalar, dy_ * scalar, dtheta_ * scalar};
}

double Twist2d::Dx() const { return dx_; }

double Twist2d::Dy() const { return dy_; }

double Twist2d::Dtheta() const { return dtheta_; }

double Twist2d::Norm() const {
  if (dy_ == 0.0) return std::abs(dx_);
  return std::hypot(dx_, dy_);
}
}
