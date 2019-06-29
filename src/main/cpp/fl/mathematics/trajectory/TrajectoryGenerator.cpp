#include "fl/mathematics/trajectory/TrajectoryGenerator.h"

namespace fl {
TimedTrajectory<Pose2dWithCurvature> TrajectoryGenerator::GenerateTrajectory(std::vector<Pose2d> waypoints,
                                                                             const Constraints&  constraints,
                                                                             const double
                                                                             start_velocity,
                                                                             const double end_velocity,
                                                                             const double max_velocity,
                                                                             const double
                                                                             max_acceleration,
                                                                             const bool reversed) {
  const auto flipped_position = Pose2d{Translation2d{}, Rotation2d::FromDegrees(180.0)};

  if (reversed) {
    for (auto& waypoint : waypoints) {
      waypoint = waypoint.TransformBy(flipped_position);
    }
  }

  const auto indexed_trajectory = TrajectoryFromSplineWaypoints(waypoints, 2.0, 0.05, 0.1);

  auto points = indexed_trajectory.Points();

  if (reversed) {
    for (auto& point : points) {
      point =
          Pose2dWithCurvature{point.Pose().TransformBy(flipped_position), -point.Curvature(), point.Dkds()};
    }
  }

  return TimeParameterizeTrajectory(DistanceTrajectory<Pose2dWithCurvature>(points), constraints,
                                    start_velocity, end_velocity, max_velocity, max_acceleration, 0.051,
                                    reversed);
}

IndexedTrajectory<Pose2dWithCurvature> TrajectoryGenerator::TrajectoryFromSplineWaypoints(
  const std::vector<Pose2d>& waypoints, const double max_dx, const double max_dy, const double max_dtheta) {
  std::vector<std::shared_ptr<ParametricSpline>> splines(waypoints.size() - 1);
  for (auto i = 1; i < waypoints.size(); ++i) {
    splines[i - 1] = std::make_shared<ParametricQuinticHermiteSpline>(waypoints[i - 1], waypoints[i]);
  }
  auto trajectory = IndexedTrajectory<Pose2dWithCurvature>(
    SplineGenerator::ParameterizeSplines(splines, max_dx, max_dy, max_dtheta));

  return trajectory;
}
}
