#include <gtest/gtest.h>
#include "FalconLibrary.h"

constexpr double kTestEpsilon = 1E-6;

class TrajectoryTest : public ::testing::Test {
 public:
  TrajectoryTest() {}

  void Run(fl::Pose2d initial,
           fl::Pose2d final,
           double max_velocity = 3,
           double max_acceleration = 2,
           bool backwards = false) {
    auto trajectory = fl::TrajectoryGenerator::GenerateTrajectory(
        std::vector<fl::Pose2d>{initial, final},
        std::vector<fl::TimingConstraint<fl::Pose2dWithCurvature>*>{},
        0.0,
        0.0,
        max_velocity,
        max_acceleration,
        backwards);

    auto pose = trajectory.Sample(units::second_t(0.0)).state.State().Pose();

    EXPECT_FALSE(false);

    EXPECT_NEAR(
        pose.Translation().X(), initial.Translation().X(), kTestEpsilon);
    EXPECT_NEAR(
        pose.Translation().Y(), initial.Translation().Y(), kTestEpsilon);
    EXPECT_NEAR(
        pose.Rotation().Radians(), initial.Rotation().Radians(), kTestEpsilon);

    const auto iterator = trajectory.Iterator();

    auto sample = iterator->Advance(0_s);

    while (!iterator->IsDone()) {
      auto prev_sample = sample;
      sample = iterator->Advance(0.02_s);

      EXPECT_LT(std::abs(units::unit_cast<double>(sample.state.Velocity())), max_velocity + kTestEpsilon);
      EXPECT_LT(std::abs(units::unit_cast<double>(sample.state.Acceleration())),
                max_acceleration + kTestEpsilon);

      if (backwards) {
        EXPECT_LT(units::unit_cast<double>(sample.state.Velocity()), 1e-9);
      } else {
        EXPECT_GT(units::unit_cast<double>(sample.state.Velocity()), -1e-9);
      }
    }

    auto pose1 = sample.state.State().Pose();

    EXPECT_NEAR(pose1.Translation().X(), final.Translation().X(), kTestEpsilon);
    EXPECT_NEAR(pose1.Translation().Y(), final.Translation().Y(), kTestEpsilon);
    EXPECT_NEAR(
        pose1.Rotation().Radians(), final.Rotation().Radians(), kTestEpsilon);
  }
};

TEST_F(TrajectoryTest, Curve) {
  Run(fl::Pose2d{0.0, 0.0, fl::Rotation2d::FromDegrees(0.0)},
      fl::Pose2d{10.0, 10.0, fl::Rotation2d::FromDegrees(50.0)});
}
