#pragma once

#include <cmath>
#include "fl/Utilities.h"

namespace fl {
class Rotation2d final {
 public:
  // Constructors
  Rotation2d();
  Rotation2d(const double x, const double y, const bool normalize);
  explicit Rotation2d(const double value);
  
  static Rotation2d FromDegrees(const double degrees);

  // Operator Overloads
  inline Rotation2d operator-(const Rotation2d& other) const { return *this + -other; }
  inline Rotation2d operator-() const { return Rotation2d(-value_); }

  inline Rotation2d operator+(const Rotation2d& other) const {
    return Rotation2d{Cos() * other.Cos() - Sin() * other.Sin(), Cos() * other.Sin() + Sin() * other.Cos(),
                      true};
  }

  // Accessors
  inline double Radians() const { return value_; }
  inline double Degrees() const { return Rad2Deg(value_); }
  inline double Cos() const { return cos_; }
  inline double Sin() const { return sin_; }
  inline double Tan() const { return sin_ / cos_; }

  bool IsParallel(const Rotation2d& other) const;

 private:
  double value_;
  double cos_;
  double sin_;
};
}  // namespace fl
