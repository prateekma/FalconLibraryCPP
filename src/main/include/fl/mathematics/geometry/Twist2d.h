#pragma once

#include <cmath>

namespace fl {
class Twist2d {
 public:
  // Constructors
  Twist2d();
  Twist2d(const double dx, const double dy, const double dtheta);

  // Operator Overloads
  Twist2d operator*(const double scalar) const;

  // Accessors
  double Dx() const;
  double Dy() const;
  double Dtheta() const;

  double Norm() const;

 private:
  double dx_;
  double dy_;
  double dtheta_;
};
}  // namespace fl
