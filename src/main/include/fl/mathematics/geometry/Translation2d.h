#pragma once

#include <cmath>

#include "Rotation2d.h"
#include "fl/types/VaryInterpolatable.h"

namespace fl {

class Translation2d final : public VaryInterpolatable<Translation2d> {
 public:
  // Constructors
  Translation2d();
  Translation2d(double x, double y);
  Translation2d(double length, const Rotation2d& rotation);

  // Overriden Methods
  double Distance(const Translation2d& other) const override;

  Translation2d Interpolate(const Translation2d& end_value, const double t) const override;

  // Operator Overloads
  Translation2d operator+(const Translation2d& other) const;

  Translation2d operator-(const Translation2d& other) const;

  Translation2d operator*(double scalar) const;

  Translation2d operator*(const Rotation2d& rotation) const;

  Translation2d operator/(double scalar) const;

  Translation2d operator-() const;

  // Accessors
  double X() const;
  double Y() const;

  double Norm() const;

 private:
  double x_;
  double y_;
};
}  // namespace fl
