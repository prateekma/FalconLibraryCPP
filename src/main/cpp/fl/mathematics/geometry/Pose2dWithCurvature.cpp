#include "fl/mathematics/geometry/Pose2dWithCurvature.h"

namespace fl {
Pose2dWithCurvature::Pose2dWithCurvature(Pose2d pose, const double curvature, const double dkds)
    : pose_(std::move(pose)), curvature_(curvature), dkds_(dkds) {}

Pose2dWithCurvature::Pose2dWithCurvature() : pose_(Pose2d{}), curvature_(0.0), dkds_(0.0) {}

double Pose2dWithCurvature::Distance(const Pose2dWithCurvature& other) const {
  return pose_.Distance(other.pose_);
}

Pose2dWithCurvature Pose2dWithCurvature::Interpolate(const Pose2dWithCurvature& end_value, double t) const {
  return Pose2dWithCurvature{pose_.Interpolate(end_value.pose_, t), Lerp(curvature_, end_value.curvature_, t),
                             Lerp(dkds_, end_value.dkds_, t)};
}

Pose2dWithCurvature Pose2dWithCurvature::operator+(const Pose2d& other) const {
  return Pose2dWithCurvature{pose_ + other, curvature_, dkds_};
}

const Pose2d& Pose2dWithCurvature::Pose() const {
  return pose_;
}

double Pose2dWithCurvature::Curvature() const {
  return curvature_;
}

double Pose2dWithCurvature::Dkds() const {
  return dkds_;
}

Pose2dWithCurvature Pose2dWithCurvature::Mirror() const {
  return Pose2dWithCurvature{pose_.Mirror(), -curvature_, -dkds_};
}

}  // namespace fl