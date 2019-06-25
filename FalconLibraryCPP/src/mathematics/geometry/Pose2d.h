#pragma once

#include "Translation2d.h"
#include "Rotation2d.h"
#include "Twist2d.h"
#include "../../Utilities.h"

namespace frc5190 {
class Twist2d;
class Pose2d final : public VaryInterpolatable<Pose2d> {
 public:
  // Constructors
  Pose2d() : translation_(Translation2d()), rotation_(Rotation2d()) {}

  Pose2d(Translation2d translation, const Rotation2d rotation)
      : translation_(std::move(translation)), rotation_(rotation) {}

  Pose2d(const double x, const double y, const Rotation2d rotation)
      : translation_(Translation2d(x, y)), rotation_(rotation) {}

  // Overriden Methods
  double Distance(const Pose2d& other) const override {
    return ToTwist(-*this + other).Norm();
  }
  Pose2d Interpolate(const Pose2d& end_value, const double t) const override {
    if (t <= 0) {
      return *this;
    }
    if (t >= 1) {
      return end_value;
    }
    const auto twist = ToTwist(-*this + end_value);
    return *this + FromTwist(twist * t);
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
    const auto twist = ToTwist(-(*this) + other);
    return EpsilonEquals(twist.Dy(), 0.0) && EpsilonEquals(twist.Dtheta(), 0.0);
  }

  // Static Methods
  static Twist2d ToTwist(const Pose2d& pose) {
    const auto dtheta = pose.rotation_.Radians();
    const auto half_dtheta = dtheta / 2.0;
    const auto cos_minus_one = pose.rotation_.Cos() - 1.0;

    double half_theta_by_tan_of_half_dtheta;

    if (std::abs(cos_minus_one) < kEpsilon) {
      half_theta_by_tan_of_half_dtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      half_theta_by_tan_of_half_dtheta =
          -(half_dtheta * pose.rotation_.Sin()) / cos_minus_one;
    }

    const auto translation_part =
        pose.translation_ *
        Rotation2d{half_theta_by_tan_of_half_dtheta, -half_dtheta, false};

    return Twist2d{translation_part.X(), translation_part.Y(),
                   pose.rotation_.Radians()};
  }

  static Pose2d FromTwist(const Twist2d& twist) {
    const auto dx = twist.Dx(), dy = twist.Dy(), dtheta = twist.Dtheta();

    const auto sin_theta = std::sin(dtheta);
    const auto cos_theta = std::cos(dtheta);

    double s, c;
    if (std::abs(dtheta) < kEpsilon) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = sin_theta / dtheta;
      c = (1 - cos_theta) / dtheta;
    }

    return Pose2d{Translation2d{dx * s - dy * c, dx * c + dy * s},
                  Rotation2d{cos_theta, sin_theta, false}};
  }

 private:
  Translation2d translation_;
  Rotation2d rotation_;
};
}  // namespace frc5190
