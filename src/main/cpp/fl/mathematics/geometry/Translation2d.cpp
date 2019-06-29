#include "fl/mathematics/geometry/Translation2d.h"

namespace fl {

Translation2d::Translation2d() : x_(0.0), y_(0.0) {}
Translation2d::Translation2d(const double x, const double y) : x_(x), y_(y) {}
Translation2d::Translation2d(const double length, const Rotation2d& rotation)
    : x_(length * rotation.Cos()), y_(length * rotation.Sin()) {}

double Translation2d::Distance(const Translation2d& other) const {
  return std::hypot(other.X() - X(), other.Y() - Y());
}

Translation2d Translation2d::Interpolate(const Translation2d& end_value, const double t) const {
  if (t <= 0) {
    return *this;
  }
  if (t >= 1) {
    return end_value;
  }
  return Translation2d{Lerp(X(), end_value.X(), t), Lerp(Y(), end_value.Y(), t)};
}

Translation2d Translation2d::operator+(const Translation2d& other) const {
  return Translation2d{X() + other.X(), Y() + other.Y()};
}

Translation2d Translation2d::operator-(const Translation2d& other) const {
  return Translation2d{X() - other.X(), Y() - other.Y()};
}

Translation2d Translation2d::operator*(const double scalar) const {
  return Translation2d{X() * scalar, Y() * scalar};
}

Translation2d Translation2d::operator*(const Rotation2d& rotation) const {
  return Translation2d{
    x_ * rotation.Cos() - y_ * rotation.Sin(),
    x_ * rotation.Sin() + y_ * rotation.Cos()
  };
}

Translation2d Translation2d::operator/(const double scalar) const {
  return Translation2d{X() / scalar, Y() / scalar};
}

Translation2d Translation2d::operator-() const { return Translation2d{-X(), -Y()}; }

double Translation2d::X() const { return x_; }

double Translation2d::Y() const { return y_; }

double Translation2d::Norm() const { return std::hypot(x_, y_); }

}  // namespace fl
