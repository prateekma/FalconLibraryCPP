#pragma once

#include <cmath>
#include "../../Utilities.h"

namespace frc5190 {
class Rotation2d final {
 public:
  // Constructors
  Rotation2d() : value_(0.0), cos_(1.0), sin_(0.0) {}
  explicit Rotation2d(const double value) : value_(value), cos_(std::cos(value)), sin_(std::sin(value)) {}

  Rotation2d(const double x, const double y, const bool normalize) {
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

  static Rotation2d FromDegrees(const double degrees) { return Rotation2d(Deg2Rad(degrees)); }

  // Operator Overloads
  Rotation2d operator-(const Rotation2d& other) const { return *this + -other; }
  Rotation2d operator-() const { return Rotation2d(-value_); }

  Rotation2d operator+(const Rotation2d& other) const {
    return Rotation2d{Cos() * other.Cos() - Sin() * other.Sin(), Cos() * other.Sin() + Sin() * other.Cos(),
                      true};
  }

  // Accessors
  double Radians() const { return value_; }
  double Degrees() const { return Rad2Deg(value_); }
  double Cos() const { return cos_; }
  double Sin() const { return sin_; }
  double Tan() const { return sin_ / cos_; }

  bool IsParallel(const Rotation2d& other) const { return EpsilonEquals((*this - other).Radians(), 0.0); }

 private:
  double value_;
  double cos_;
  double sin_;
};
}  // namespace frc5190
