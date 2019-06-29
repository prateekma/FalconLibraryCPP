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
                                                             const double t1 = 1.0) {
    const auto dt = t1 - t0;
    auto       rv = std::vector<Pose2dWithCurvature>(static_cast<int>(kMinSampleSize / dt));

    rv.push_back(spline->PoseWithCurvature(0));

    for (double t = 0; t < t1; t += dt / kMinSampleSize) {
      GetSegmentArc(spline, &rv, t, t + dt / kMinSampleSize, max_dx, max_dy, max_dtheta);
    }

    return rv;
  }

  static std::vector<Pose2dWithCurvature> ParameterizeSplines(
      std::vector<std::shared_ptr<ParametricSpline>> splines, const double max_dx, const double max_dy,
      const double max_dtheta) {
    auto rv = std::vector<Pose2dWithCurvature>();
    if (splines.empty()) return rv;

    rv.push_back(splines[0]->PoseWithCurvature(0.0));
    for (const auto& spline : splines) {
      auto samples = ParameterizeSpline(spline, max_dx, max_dy, max_dtheta);
      samples.erase(samples.begin());
      rv.insert(rv.end(), samples.begin(), samples.end());
    }

    return rv;
  }

  static void GetSegmentArc(const std::shared_ptr<ParametricSpline>& spline,
                            std::vector<Pose2dWithCurvature>* rv, const double t0, const double t1,
                            const double max_dx, const double max_dy, const double max_dtheta) {
    const auto p0 = spline->Point(t0);
    const auto p1 = spline->Point(t1);
    const auto r0 = spline->Heading(t0);
    const auto r1 = spline->Heading(t1);

    const auto transformation = Pose2d{(p1 - p0) * -r0, r1 + -r0};
    const auto twist          = Pose2d::ToTwist(transformation);

    if (twist.Dy() > max_dy || twist.Dx() > max_dx || twist.Dtheta() > max_dtheta) {
      GetSegmentArc(spline, rv, t0, (t0 + t1) / 2, max_dx, max_dy, max_dtheta);
      GetSegmentArc(spline, rv, (t0 + t1) / 2, t1, max_dx, max_dy, max_dtheta);
    } else {
      rv->push_back(spline->PoseWithCurvature(t1));
    }
  }
};
}  // namespace fl
