#include "fl/mathematics/geometry/Rotation2d.h"

namespace fl {

Rotation2d::Rotation2d() : value_(0.0), cos_(1.0), sin_(0.0) {}
Rotation2d::Rotation2d(const double value) : value_(value), cos_(std::cos(value)), sin_(std::sin(value)) {}
Rotation2d::Rotation2d(const double x, const double y, const bool normalize) {
  if (normalize) {
    const auto magnitude = std::hypot(x, y);
    if (magnitude > kEpsilon) {
      sin_ = y / magnitude;
      cos_ = x / magnitude;
    } else {
      sin_ = 0.0;
      cos_ = 1.0;
    }
  } else {
    cos_ = x;
    sin_ = y;
  }
  value_ = std::atan2(sin_, cos_);
}

Rotation2d Rotation2d::FromDegrees(const double degrees) {
  return Rotation2d(Deg2Rad(degrees));
}

bool Rotation2d::IsParallel(const Rotation2d& other) const {
  return EpsilonEquals((*this - other).Radians(), 0.0);
}

}  // namespace fl