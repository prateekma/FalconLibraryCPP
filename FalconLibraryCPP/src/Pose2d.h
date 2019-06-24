#pragma once

#include "VaryInterpolatable.h"
#include "Translation2d.h"
#include "Rotation2d.h"
#include "Twist2d.h"
#include "Utilities.h"

namespace frc5190 {
class Pose2d final : public VaryInterpolatable<Pose2d> {
 public:
  // Constructors
  Pose2d() : translation_(Translation2d()), rotation_(Rotation2d()) {}

  Pose2d(Translation2d translation, const Rotation2d rotation)
      : translation_(std::move(translation)), rotation_(rotation) {}

  Pose2d(const double x, const double y, const Rotation2d rotation)
      : translation_(Translation2d(x, y)), rotation_(rotation) {}

  // Overriden Methods
  double Distance(const Pose2d& other) override {
    return (-*this + other).AsTwist().Norm();
  }
  Pose2d Interpolate(const Pose2d& end_value, const double t) override {
    if (t <= 0) {
      return *this;
    }
    if (t >= 1) {
      return end_value;
    }
    const auto twist = (-*this + end_value).AsTwist();
    return *this + (twist * t).AsPose();
  }

  // Operator Overloads
  Pose2d operator+(const Pose2d& other) const { return TransformBy(other); }
  Pose2d operator-(const Pose2d& other) const { return TransformBy(-other); }
  Pose2d operator-() const {
    const auto inverted_rotation = -rotation_;
    return Pose2d{(-translation_) * inverted_rotation, inverted_rotation};
  }

  // Accessors
  const Translation2d& Translation() const { return translation_; }
  const Rotation2d& Rotation() const { return rotation_; }

  Twist2d AsTwist() const {
    const auto dtheta = rotation_.Radians();
    const auto half_dtheta = dtheta / 2.0;
    const auto cos_minus_one = rotation_.Cos() - 1.0;

    double half_theta_by_tan_of_half_dtheta;

    if (std::abs(cos_minus_one) < kEpsilon) {
      half_theta_by_tan_of_half_dtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      half_theta_by_tan_of_half_dtheta =
          -(half_dtheta * rotation_.Sin()) / cos_minus_one;
    }

    const auto translation_part =
        translation_ *
        Rotation2d{half_theta_by_tan_of_half_dtheta, -half_dtheta, false};

    return Twist2d{translation_part.X(), translation_part.Y(),
                   rotation_.Radians()};
  }

  Pose2d Mirror() const {
    return Pose2d{Translation2d{translation_.X(), 27.0 - translation_.Y()},
                  -rotation_};
  }

  Pose2d TransformBy(const Pose2d& other) const {
    return Pose2d{translation_ + (other.translation_ * rotation_),
                  rotation_ + other.rotation_};
  }

  bool IsCollinear(const Pose2d& other) const {
    if (!rotation_.IsParallel(other.rotation_)) {
      return false;
    }
    const auto twist = (-(*this) + other).AsTwist();
    return EpsilonEquals(twist.Dy(), 0.0) && EpsilonEquals(twist.Dtheta(), 0.0);
  }

 private:
  Translation2d translation_;
  Rotation2d rotation_;
};
}  // namespace frc5190
