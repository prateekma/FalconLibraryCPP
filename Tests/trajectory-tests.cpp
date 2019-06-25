#include "pch.h"
#include "../FalconLibraryCPP/src/FalconLibrary.h"
#include "../FalconLibraryCPP/src/mathematics/trajectory/constraints/CentripetalAccelerationConstraint.h"

constexpr double kTestEpsilon = 1E-6;

class TrajectoryTest : public ::testing::Test {
 public:
  TrajectoryTest() {}

  void Run(frc5190::Pose2d initial, frc5190::Pose2d final,
           double max_velocity = 3, double max_acceleration = 2,
           bool backwards = false) {
    auto trajectory = frc5190::TrajectoryGenerator::GenerateTrajectory(
        std::vector<frc5190::Pose2d>{initial, final},
        std::vector<frc5190::TimingConstraint<frc5190::Pose2dWithCurvature>*>{new frc5190::CentripetalAccelerationConstraint{100.0}},
        0.0, 0.0, max_velocity, max_acceleration, backwards);

    auto pose = trajectory.Sample(0.0).state.State().Pose();

    /*
    EXPECT_NEAR(pose.Translation().X(), initial.Translation().X(),
                kTestEpsilon);
    EXPECT_NEAR(pose.Translation().Y(), initial.Translation().Y(),
                kTestEpsilon);
    EXPECT_NEAR(pose.Rotation().Radians(), initial.Rotation().Degrees(),
                kTestEpsilon);
    */

    /*
    const auto iterator = trajectory.Iterator();

    auto sample = iterator->Advance(0.0);

    while (!iterator->IsDone()) {
      auto prev_sample = sample;
      sample = iterator->Advance(0.02);

      EXPECT_LT(std::abs(sample.state.Velocity()), max_velocity + kTestEpsilon);
      EXPECT_LT(std::abs(sample.state.Acceleration()),
                max_acceleration + kTestEpsilon);

      if (backwards) {
        EXPECT_LT(sample.state.Velocity(), 1e-9);
      } else {
        EXPECT_GT(sample.state.Velocity(), -1e-9);
      }
    }

    auto pose1 = sample.state.State().Pose();

    EXPECT_NEAR(pose1.Translation().X(), final.Translation().X(), kTestEpsilon);
    EXPECT_NEAR(pose1.Translation().Y(), final.Translation().Y(), kTestEpsilon);
    EXPECT_NEAR(pose1.Rotation().Radians(), final.Rotation().Degrees(),
                kTestEpsilon);
      */
  }
};

TEST_F(TrajectoryTest, Curve) {
  Run(frc5190::Pose2d{0.0, 0.0, frc5190::Rotation2d::FromDegrees(0.0)},
      frc5190::Pose2d{10.0, 10.0, frc5190::Rotation2d::FromDegrees(50.0)});
}
