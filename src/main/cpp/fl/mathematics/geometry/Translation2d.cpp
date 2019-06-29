#include "fl/mathematics/geometry/Translation2d.h"

namespace fl {

Translation2d::Translation2d() : x_(0.0), y_(0.0) {}
Translation2d::Translation2d(const double x, const double y) : x_(x), y_(y) {}
Translation2d::Translation2d(const double length, const Rotation2d& rotation)
    : x_(length * rotation.Cos()), y_(length * rotation.Sin()) {}

Translation2d Translation2d::Interpolate(const Translation2d& end_value, const double t) const {
  if (t <= 0) {
    return *this;
  }
  if (t >= 1) {
    return end_value;
  }
  return Translation2d{Lerp(X(), end_value.X(), t), Lerp(Y(), end_value.Y(), t)};
}

}  // namespace fl
