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
      const std::vector<TimingConstraint<Pose2dWithCurvature>*>& constraints,
      const double start_velocity, const double end_velocity,
      const double max_velocity, const double max_acceleration, bool reversed) {
    const auto flipped_position =
        Pose2d{Translation2d{}, Rotation2d::FromDegrees(180.0)};

    if (reversed) {
      for (auto& waypoint : waypoints) {
        waypoint = waypoint.TransformBy(flipped_position);
      }
    }

    const auto indexed_trajectory =
        TrajectoryFromSplineWaypoints(waypoints, 0.051, 0.00127, 0.1);

    auto points = indexed_trajectory.Points();

    if (reversed) {
      for (auto& point : points) {
        point = Pose2dWithCurvature{point.Pose().TransformBy(flipped_position),
                                    -point.Curvature(), point.Dkds()};
      }
    }

    const auto trajectory = IndexedTrajectory<Pose2dWithCurvature>(points);

    return TimeParameterizeTrajectory(
        DistanceTrajectory<Pose2dWithCurvature>(trajectory.Points()),
        constraints, start_velocity, end_velocity, max_velocity,
        max_acceleration, 0.051, reversed);
  }

  static IndexedTrajectory<Pose2dWithCurvature> TrajectoryFromSplineWaypoints(
      const std::vector<Pose2d>& waypoints, const double max_dx,
      const double max_dy, const double max_dtheta) {
    auto size = static_cast<int>(waypoints.size());
    std::vector<ParametricSpline*> splines(size - 1);
    for (auto i = 1; i < waypoints.size(); ++i) {
      splines[i - 1] =
          new ParametricQuinticHermiteSpline(waypoints[i - 1], waypoints[i]);
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
  struct ConstrainedPose {
    S state;
    double distance;
    double max_velocity;
    double min_acceleration;
    double max_acceleration;
  };

  template <typename S>
  static void EnforceAccelerationLimits(
      bool reverse, std::vector<TimingConstraint<S>*> constraints,
      ConstrainedPose<S>* constrained_pose) {
    for (const auto& constraint : constraints) {
      auto min_max_accel = constraint->MinMaxAcceleration(
          constrained_pose->state,
          reverse ? -1.0 : 1.0 * constrained_pose->max_velocity);

      if (!min_max_accel.IsValid()) throw - 1;

      constrained_pose->min_acceleration =
          std::max(constrained_pose->min_acceleration,
                   reverse ? -min_max_accel.max_acceleration
                           : min_max_accel.min_acceleration);

      constrained_pose->max_acceleration =
          std::min(constrained_pose->max_acceleration,
                   reverse ? -min_max_accel.min_acceleration
                           : min_max_accel.max_acceleration);
    }
  }

  template <typename S>
  static TimedTrajectory<S> TimeParameterizeTrajectory(
      DistanceTrajectory<S> distance_trajectory,
      std::vector<TimingConstraint<Pose2dWithCurvature>*> constraints,
      double start_velocity, double end_velocity, double max_velocity,
      double max_acceleration, double step_size, bool reversed) {

    const auto num_states = static_cast<int>(
        std::ceil(distance_trajectory.LastInterpolant() / step_size + 1));

    constexpr static auto epsilon = 1E-6;
    static auto last = distance_trajectory.LastInterpolant();

    std::vector<S> states(num_states);
    for (auto i = 0; i < num_states; ++i) {
      states[i] = distance_trajectory
                      .Sample(std::min(i * step_size,
                                       last))
                      .state;
    }

    // Forward pass. We look at pairs of consecutive states, where the start
    // state has already been velocity parameterized (though we may adjust
    // the velocity downwards during the backwards pass). We wish to find an
    // acceleration that is admissible at both the start and end state, as
    // well as an admissible end velocity. If there is no admissible end
    // velocity or acceleration, we set the end velocity to the state's
    // maximum allowed velocity and will repair the acceleration during the
    // backward pass (by slowing down the predecessor).

    std::vector<ConstrainedPose<S>> constrained_poses(num_states);

    auto _predecessor = ConstrainedPose<S>{states[0], 0.0, start_velocity,
                                           -max_acceleration, max_acceleration};
    ConstrainedPose<S>* predecessor = &_predecessor;

    for (auto i = 0; i < states.size(); ++i) {
      constrained_poses[i] = ConstrainedPose<S>{};
      ConstrainedPose<S>& constrained_pose = constrained_poses.at(i);

      constrained_pose.state = states.at(i);
      double ds = constrained_pose.state.Distance(predecessor->state);
      constrained_pose.distance = ds + predecessor->distance;

      // We may need to iterate to find the maximum end velocity and common
      // acceleration, since acceleration limits may be a function of velocity.
      while (true) {
        // Enforce global max velocity and max reachable velocity by global
        // acceleration limit. vf = sqrt(vi^2 + 2*a*d)
        constrained_pose.max_velocity = std::min(
            max_velocity,
            std::sqrt(predecessor->max_velocity * predecessor->max_velocity +
                      2.0 * predecessor->max_acceleration * ds));

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
              std::min(constraint->MaxVelocity(constrained_pose.state),
                       constrained_pose.max_velocity);
        }

        if (constrained_pose.max_velocity < 0.0) throw - 1;

        // Now enforce all acceleration constraints.
        EnforceAccelerationLimits(reversed, constraints, &constrained_pose);

        if (ds < epsilon) break;

        // If the max acceleration for this constraint state is more
        // conservative than what we had applied, we need to reduce the max
        // accel at the predecessor state and try again.
        auto actual_acceleration = (std::pow(constrained_pose.max_velocity, 2) -
                                    std::pow(predecessor->max_velocity, 2)) /
                                   (2.0 * ds);

        if (constrained_pose.max_acceleration < actual_acceleration - epsilon) {
          predecessor->max_acceleration = constrained_pose.max_acceleration;
        } else {
          if (actual_acceleration > predecessor->min_acceleration + epsilon) {
            predecessor->max_acceleration = actual_acceleration;
          }
          break;
        }
      }
      predecessor = &constrained_pose;
    }

    // Backward pass
    auto _successor =
        ConstrainedPose<S>{states[states.size() - 1],
                           constrained_poses[states.size() - 1].distance,
                           end_velocity, -max_acceleration, max_acceleration};
    ConstrainedPose<S>* successor = &_successor;

    for (auto i = states.size() - 1; i >= 0; --i) {
      auto state = constrained_poses.at(i);
      const auto ds = state.distance - successor->distance;  // will be negative

      while (true) {
        // Enforce reverse max reachable velocity limit.
        // vf = sqrt(vi^2 + 2*a*d), where vi = successor.

        const auto new_max_velocity =
            std::sqrt(successor->max_velocity * successor->max_velocity +
                      2.0 * successor->min_acceleration * ds);

        if (new_max_velocity >= state.max_velocity) {
          break;
        }

        state.max_velocity = new_max_velocity;
        if (std::isnan(new_max_velocity)) {
          throw - 1;
        }

        // Now check all acceleration constraints with the lower max velocity.
        EnforceAccelerationLimits(reversed, constraints, &state);

        if (ds > epsilon) break;

        // If the min acceleration for this constraint state is more
        // conservative than what we have applied, we need to reduce the min
        // accel and try again.

        auto actual_acceleration = (std::pow(state.max_velocity, 2) -
                                    std::pow(successor->max_velocity, 2)) /
                                   (2 * ds);

        if (state.min_acceleration > actual_acceleration + epsilon) {
          successor->min_acceleration = state.min_acceleration;
        } else {
          successor->min_acceleration = actual_acceleration;
          break;
        }
      }
      successor = &state;
    }

    std::vector<TimedEntry<S>> timed_states(states.size(), TimedEntry<S>());

    auto t = 0.;
    auto s = 0.;
    auto v = 0.;

    for (auto i = 0; i < states.size(); i++) {
      const ConstrainedPose<S> constrained_pose = constrained_poses.at(i);
      const double ds = constrained_pose.distance - s;
      double accel =
          (constrained_pose.max_velocity * constrained_pose.max_velocity -
           v * v) /
          (2. * ds);
      double dt = 0.;
      if (i > 0) {
        timed_states.at(i - 1).SetAcceleration(reversed ? -accel : accel);
        if (std::abs(accel) > 1e-6) {
          dt = (constrained_pose.max_velocity - v) / accel;
        } else if (std::abs(v) > 1e-6) {
          dt = ds / v;
        }
      }

      v = constrained_pose.max_velocity;
      s = constrained_pose.distance;
      timed_states[i] = TimedEntry<S>{constrained_pose.state, t,
                                           reversed ? -v : v,
                                           reversed ? -accel : accel};

      t += dt;
    }

    return TimedTrajectory<S>(timed_states, reversed);
  }
};
}  // namespace frc5190
