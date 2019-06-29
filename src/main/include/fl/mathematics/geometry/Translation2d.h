#pragma once

#include <cmath>

#include "Rotation2d.h"
#include "fl/types/VaryInterpolatable.h"

namespace fl {

class Translation2d final : public VaryInterpolatable<Translation2d> {
 public:
  // Constructors
  Translation2d();
  Translation2d(const double x, const double y);
  Translation2d(const double length, const Rotation2d& rotation);

  // Overriden Methods
  inline double Distance(const Translation2d& other) const override {
    return std::hypot(other.X() - X(), other.Y() - Y());
  }

  Translation2d Interpolate(const Translation2d& end_value, const double t) const override;

  // Operator Overloads
  inline Translation2d operator+(const Translation2d& other) const {
    return Translation2d{X() + other.X(), Y() + other.Y()};
  }

  inline Translation2d operator-(const Translation2d& other) const {
    return Translation2d{X() - other.X(), Y() - other.Y()};
  }

  inline Translation2d operator*(const double scalar) const {
    return Translation2d{X() * scalar, Y() * scalar};
  }

  inline Translation2d operator*(const Rotation2d& rotation) const {
    return Translation2d{x_ * rotation.Cos() - y_ * rotation.Sin(),
                         x_ * rotation.Sin() + y_ * rotation.Cos()};
  }

  inline Translation2d operator/(const double scalar) const {
    return Translation2d{X() / scalar, Y() / scalar};
  }

  inline Translation2d operator-() const { return Translation2d{-X(), -Y()}; }

  // Accessors
  inline double X() const { return x_; }
  inline double Y() const { return y_; }

  inline double Norm() const { return std::hypot(x_, y_); }

 private:
  double x_;
  double y_;
};
}  // namespace fl
