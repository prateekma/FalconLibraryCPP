#pragma once

#include <memory>
#include <vector>

#include "ParametricSpline.h"
#include "fl/mathematics/geometry/Pose2dWithCurvature.h"

namespace fl {
constexpr static double kMinSampleSize = 1.;

class SplineGenerator {
 public:
  static std::vector<Pose2dWithCurvature> ParameterizeSpline(const std::shared_ptr<ParametricSpline>& spline,
                                                             const double max_dx, const double max_dy,
                                                             const double max_dtheta, const double t0 = 0.0,
                                                             const double t1 = 1.0);

  static std::vector<Pose2dWithCurvature> ParameterizeSplines(
      std::vector<std::shared_ptr<ParametricSpline>> splines, const double max_dx, const double max_dy,
      const double max_dtheta);

  static void GetSegmentArc(const std::shared_ptr<ParametricSpline>& spline,
                            std::vector<Pose2dWithCurvature>* rv, const double t0, const double t1,
                            const double max_dx, const double max_dy, const double max_dtheta);
};
}  // namespace fl
