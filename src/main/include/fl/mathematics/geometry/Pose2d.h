#pragma once

#include "Rotation2d.h"
#include "Translation2d.h"
#include "Twist2d.h"

namespace fl {
class Pose2d final : public VaryInterpolatable<Pose2d> {
 public:
  // Constructors
  Pose2d();
  Pose2d(Translation2d translation, const Rotation2d rotation);
  Pose2d(const double x, const double y, const Rotation2d rotation);

  // Overriden Methods
  double Distance(const Pose2d& other) const override;
  Pose2d Interpolate(const Pose2d& end_value, const double t) const override;

  // Operator Overloads
  Pose2d operator+(const Pose2d& other) const;
  Pose2d operator-(const Pose2d& other) const;
  Pose2d operator-() const;

  // Accessors
  const Translation2d& Translation() const;
  const Rotation2d&    Rotation() const;

  // Member Functions
  Pose2d Mirror() const;
  Pose2d TransformBy(const Pose2d& other) const;
  bool IsCollinear(const Pose2d& other) const;
  Pose2d InFrameOfReferenceOf(const Pose2d& other) const;

  // Static Methods
  static Twist2d ToTwist(const Pose2d& pose);
  static Pose2d FromTwist(const Twist2d& twist);

 private:
  Translation2d translation_;
  Rotation2d    rotation_;
};
}  // namespace fl
