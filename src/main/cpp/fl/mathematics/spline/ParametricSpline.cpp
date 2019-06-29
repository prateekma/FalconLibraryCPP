#include "fl/mathematics/spline/ParametricSpline.h"

namespace fl {

Pose2dWithCurvature ParametricSpline::PoseWithCurvature(const double t) const {
  return Pose2dWithCurvature{Pose(t), Curvature(t), DCurvature(t) / Velocity(t)};
}

Pose2d ParametricSpline::Pose(const double t) const {
  return Pose2d{Point(t), Heading(t)};
}

}  // namespace fl
