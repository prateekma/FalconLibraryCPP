#include "pch.h"
#include "../FalconLibraryCPP/src/FalconLibrary.h"

class RamseteTest : public ::testing::Test {
 public:
  void Run(frc5190::Pose2d initial, frc5190::Pose2d final, double max_velocity = 3,
           double max_acceleration = 2, bool backwards = false) {
    auto trajectory = frc5190::TrajectoryGenerator::GenerateTrajectory(
        std::vector<frc5190::Pose2d>{initial, final},
        std::vector<frc5190::TimingConstraint<frc5190::Pose2dWithCurvature>*>{}, 0.0, 0.0, max_velocity,
        max_acceleration, backwards);

    frc5190::RamseteTracker tracker{2.0, 0.7};
    tracker.Reset(trajectory);
  }
};

TEST_F(RamseteTest, Test) {
  Run(frc5190::Pose2d{0.0, 0.0, frc5190::Rotation2d::FromDegrees(0.0)},
      frc5190::Pose2d{10.0, 10.0, frc5190::Rotation2d::FromDegrees(50.0)});
}