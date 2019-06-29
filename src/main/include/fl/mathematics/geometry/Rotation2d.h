#pragma once

#include <cmath>
#include "fl/Utilities.h"

namespace fl {
class Rotation2d final {
 public:
  // Constructors
  Rotation2d();
  Rotation2d(double x, double y, bool normalize);
  explicit Rotation2d(const double value);

  static Rotation2d FromDegrees(double degrees);

  // Operator Overloads
  Rotation2d operator-(const Rotation2d& other) const;
  Rotation2d operator-() const;

  Rotation2d operator+(const Rotation2d& other) const;

  // Accessors
  double Radians() const;
  double Degrees() const;
  double Cos() const;
  double Sin() const;
  double Tan() const;

  bool IsParallel(const Rotation2d& other) const;

 private:
  double value_;
  double cos_;
  double sin_;
};
}  // namespace fl
