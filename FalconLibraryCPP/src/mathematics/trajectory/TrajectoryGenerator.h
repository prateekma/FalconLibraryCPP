#pragma once

#include <array>
#include "../geometry/Pose2dWithCurvature.h"
#include "../spline/ParametricQuinticHermiteSpline.h"
#include "../spline/SplineGenerator.h"
#include "DistanceTrajectory.h"
#include "IndexedTrajectory.h"
#include "TimedTrajectory.h"
#include "constraints/TimingConstraint.h"

namespace frc5190 {

class TrajectoryGenerator {
 public:
  static TimedTrajectory<Pose2dWithCurvature> GenerateTrajectory(
      std::vector<Pose2d> waypoints,
      const std::vector<TimingConstraint<Pose2dWithCurvature>>& constraints,
      double start_velocity, double end_velocity, double max_velocity,
      double max_acceleration, bool reversed) {
    const auto flipped_position =
        Pose2d{Translation2d{}, Rotation2d::FromDegrees(180.0)};

    if (reversed) {
      for (auto& waypoint : waypoints) {
        waypoint = waypoint.TransformBy(flipped_position);
      }
    }

    auto points =
        TrajectoryFromSplineWaypoints(waypoints, 0.051, 0.00127, 0.1).Points();

    if (reversed) {
      for (auto& point : points) {
        point = Pose2dWithCurvature{point.Pose().TransformBy(flipped_position),
                                    -point.Curvature(), point.Dkds()};
      }
    }

    auto trajectory = IndexedTrajectory<Pose2dWithCurvature>(points);
  }

  static IndexedTrajectory<Pose2dWithCurvature> TrajectoryFromSplineWaypoints(
      const std::vector<Pose2d>& waypoints, const double max_dx,
      const double max_dy, const double max_dtheta) {
    std::vector<ParametricSpline*> splines(waypoints.size() - 1);
    for (auto i = 1; i < waypoints.size(); ++i) {
      splines.push_back(
          new ParametricQuinticHermiteSpline(waypoints[i - 1], waypoints[i]));
    }
    auto trajectory = IndexedTrajectory<Pose2dWithCurvature>(
        SplineGenerator::ParameterizeSplines(splines, max_dx, max_dy,
                                             max_dtheta));

    for (auto ptr : splines) {
      delete ptr;
    }

    return trajectory;
  }

  template <typename S>
  static TimedTrajectory<S> TimeParameterizeTrajectory(
      DistanceTrajectory<S> distance_trajectory,
      const std::vector<TimingConstraint<Pose2dWithCurvature>>& constraints,
      double start_velocity, double end_velocity, double max_velocity,
      double max_acceleration, double step_size, bool reversed) {
    const auto num_states = static_cast<int>(
        std::ceil(distance_trajectory.LastInterpolant() / step_size + 1));

    std::vector<S> states(num_states);
    for (auto i = 0; i < num_states; ++i) {
      states.push_back(
          distance_trajectory
              .Sample(std::min(i * step_size,
                               distance_trajectory.LastInterpolant()))
              .state);
    }

    struct ConstrainedPose {
      S state;
      double distance;
      double max_velocity;
      double min_acceleration;
      double max_acceleration;
    };

    // Forward pass. We look at pairs of consecutive states, where the start
    // state has already been velocity parameterized (though we may adjust the
    // velocity downwards during the backwards pass). We wish to find an
    // acceleration that is admissible at both the start and end state, as well
    // as an admissible end velocity. If there is no admissible end velocity or
    // acceleration, we set the end velocity to the state's maximum allowed
    // velocity and will repair the acceleration during the backward pass (by
    // slowing down the predecessor).

    std::array<ConstrainedPose, states.size()> constrained_poses;

    ConstrainedPose predecessor{states[0], 0.0, start_velocity,
                                -max_acceleration, max_acceleration};

    constrained_poses.at(0) = predecessor;

    for (auto i = 0; i < states.size(); ++i) {
      ConstrainedPose& constrained_pose = constrained_poses.at(i);

      constrained_pose.state = states.at(i);
      double ds = constrained_pose.state.Distance(predecessor.state);
      constrained_pose.distance = ds + predecessor.distance;

      // We may need to iterate to find the maximum end velocity and common
      // acceleration, since acceleration limits may be a function of velocity.
      while (true) {
        // Enforce global max velocity and max reachable velocity by global
        // acceleration limit. vf = sqrt(vi^2 + 2*a*d)
        constrained_pose.max_velocity = std::min(
            max_velocity,
            std::sqrt(predecessor.max_velocity * predecessor.max_velocity +
                      2.0 * predecessor.max_acceleration * ds));

        if (std::isnan(constrained_pose.max_velocity)) {
          throw - 1;
        }

        constrained_pose.min_acceleration = -max_acceleration;
        constrained_pose.max_acceleration = max_acceleration;

        // At this point, the state is full constructed, but no constraints have
        // been applied aside from predecessor state max accel.

        // Enforce all velocity constraints.

        for (const auto& constraint : constraints) {
          constrained_pose.max_velocity =
              std::min(constraint.MaxVelocity(constrained_pose.state),
                       constrained_pose.max_velocity);
        }

        if (constrained_pose.max_velocity < 0.0) throw -1;


      }
    }
  }
};
}  // namespace frc5190
