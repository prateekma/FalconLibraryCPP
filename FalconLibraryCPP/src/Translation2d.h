#pragma once

#include <cmath>

#include "Interpolatable.h"
#include "VaryInterpolatable.h"
#include "Rotation2d.h"

namespace frc5190 {

class Translation2d final : public VaryInterpolatable<Translation2d> {
 public:
  // Constructors
  Translation2d() : x_(0.0), y_(0.0) {}
  Translation2d(const double x, const double y) : x_(x), y_(y) {}
  Translation2d(const double distance, const Rotation2d& rotation)
      : x_(distance * rotation.Cos()), y_(distance * rotation.Sin()) {}

  // Overriden Methods
  double Distance(const Translation2d& other) override {
    return std::hypot(other.X() - X(), other.Y() - Y());
  }

  Translation2d Interpolate(const Translation2d& end_value,
                            const double t) override {
    if (t <= 0) {
      return *this;
    }
    if (t >= 1) {
      return end_value;
    }
    return Translation2d{Lerp(X(), end_value.X(), t),
                         Lerp(Y(), end_value.Y(), t)};
  }

  // Operator Overloads
  Translation2d operator+(const Translation2d& other) const {
    return Translation2d{X() + other.X(), Y() + other.Y()};
  }

  Translation2d operator-(const Translation2d& other) const {
    return Translation2d{X() - other.X(), Y() - other.Y()};
  }

  Translation2d operator*(const double scalar) const {
    return Translation2d{X() * scalar, Y() * scalar};
  }

  Translation2d operator*(const Rotation2d& rotation) const {
    return Translation2d{x_ * rotation.Cos() - y_ * rotation.Sin(),
                         x_ * rotation.Sin() + y_ * rotation.Cos()};
  }

      Translation2d
      operator/(const double scalar) const {
    return Translation2d{X() / scalar, Y() / scalar};
  }

  Translation2d operator-() const { return Translation2d{-X(), -Y()}; }

  // Accessors
  double X() const { return x_; }
  double Y() const { return y_; }

 private:
  double x_;
  double y_;
};
}  // namespace frc5190
