#include <gtest/gtest.h>
#include "FalconLibrary.h"

constexpr double kTestEpsilon = 1E-9;

class PurePursuitTest : public ::testing::Test {
 public:
  void Run(fl::Pose2d initial, fl::Pose2d final, double max_velocity = 3, double max_acceleration = 2,
           bool backwards = false) {
    auto trajectory = fl::TrajectoryGenerator::GenerateTrajectory(
        std::vector<fl::Pose2d>{initial, final},
        std::vector<fl::TimingConstraint<fl::Pose2dWithCurvature>*>{}, 0.0, 0.0, max_velocity,
        max_acceleration, backwards);

    fl::PurePursuitTracker tracker{3.0, 2.0_s, 0.3};
    tracker.Reset(trajectory);

    fl::Pose2d      robot_pose = initial;
    units::second_t t          = 0_s;

    while (!tracker.IsFinished()) {
      const auto output = tracker.NextState(robot_pose, t);

      fl::Twist2d twist{output.linear_velocity * 0.02, 0.0, output.angular_velocity * 0.02};
      robot_pose = robot_pose + fl::Pose2d::FromTwist(twist);

      t += 20_ms;
    }

    EXPECT_NEAR(robot_pose.Translation().X(), final.Translation().X(), 0.1);
    EXPECT_NEAR(robot_pose.Translation().Y(), final.Translation().Y(), 0.1);
    EXPECT_NEAR(robot_pose.Rotation().Radians(), final.Rotation().Radians(), 0.15);
  }
};

TEST_F(PurePursuitTest, Test) {
  Run(fl::Pose2d{0.0, 0.0, fl::Rotation2d::FromDegrees(0.0)},
      fl::Pose2d{10.0, 10.0, fl::Rotation2d::FromDegrees(50.0)});
}