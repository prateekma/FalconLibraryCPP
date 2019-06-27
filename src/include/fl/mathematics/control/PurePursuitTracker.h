#pragma once

#include "TrajectoryTracker.h"
#include "fl/mathematics/geometry/Pose2d.h"
#include "fl/mathematics/geometry/Pose2dWithCurvature.h"

namespace fl {
class PurePursuitTracker : public TrajectoryTracker {
 public:
  PurePursuitTracker(const double lat, const double lookahead_time, const double min_lookahead_distance)
      : lat_(lat), lookahead_time_(lookahead_time), min_lookahead_distance_(min_lookahead_distance) {}

  TrajectoryTrackerVelocityOutput CalculateState(const TimedIterator<Pose2dWithCurvature>& iterator,
                                                 const Pose2d& robot_pose) const override {
    const auto reference_point = iterator.CurrentState();

    // Compute the lookahead state.
    const auto lookahead_state = CalculateLookaheadPose(iterator, robot_pose);

    // Find the appropriate lookahead point.
    const auto lookahead_transform = lookahead_state.InFrameOfReferenceOf(robot_pose);

    // Calculate latitude error.
    const auto x_error =
        reference_point.state.State().Pose().InFrameOfReferenceOf(robot_pose).Translation().X();

    // Calculate the velocity at the reference point.
    const auto vd = reference_point.state.Velocity();

    // Calculate the distance from the robot to the lookahead.
    const auto l = lookahead_transform.Translation().Norm();

    // Calculate the curvature of the arc that connects the robot and the lookahead point.
    const auto curvature = 2 * lookahead_transform.Translation().Y() / std::pow(l, 2);

    // Adjust the linear velocity to compensate for the robot lagging behind.
    const auto adjusted_linear_velocity = vd * lookahead_transform.Rotation().Cos() + lat_ * x_error;

    return {adjusted_linear_velocity, adjusted_linear_velocity * curvature};
  }

 private:
  double lat_;
  double lookahead_time_;
  double min_lookahead_distance_;

  Pose2d CalculateLookaheadPose(const TimedIterator<Pose2dWithCurvature>& iterator,
                                const Pose2d&                             robot_pose) const {
    auto lookahead_pose_by_time = iterator.Preview(lookahead_time_).state.State().Pose();

    // The lookahead point is farther from the robot than the minimum lookahead distance.
    // Therefore we can use this point.
    if (lookahead_pose_by_time.InFrameOfReferenceOf(robot_pose).Translation().Norm() >=
        min_lookahead_distance_) {
      return lookahead_pose_by_time;
    }

    auto lookahead_pose_by_distance = iterator.CurrentState().state.State().Pose();
    auto previewed_time             = 0.0;

    // Run the loop until a distance that is greater than the minimum lookahead distance is found or until
    // we run out of "trajectory" to search. If this happens, we will simply extend the end of the trajectory.
    while (iterator.RemainingProgress() > previewed_time) {
      previewed_time += 0.02;
      lookahead_pose_by_distance = iterator.Preview(previewed_time).state.State().Pose();

      const auto lookahead_distance =
          lookahead_pose_by_distance.InFrameOfReferenceOf(robot_pose).Translation().Norm();

      if (lookahead_distance > min_lookahead_distance_) {
        return lookahead_pose_by_distance;
      }
    }

    const auto remaining = min_lookahead_distance_ -
                           (lookahead_pose_by_distance.InFrameOfReferenceOf(robot_pose)).Translation().Norm();

    // Extend the trajectory
    return lookahead_pose_by_distance.TransformBy(
        Pose2d{Translation2d{remaining * (iterator.Reversed() ? -1.0 : 1.0), 0.0}, Rotation2d{}});
  }
};
}  // namespace fl