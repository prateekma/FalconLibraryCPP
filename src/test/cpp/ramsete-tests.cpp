#include <gtest/gtest.h>
#include "FalconLibrary.h"

class RamseteTest : public ::testing::Test {
 public:
  void Run(fl::Pose2d initial, fl::Pose2d final, double max_velocity = 3,
           double max_acceleration = 2, bool backwards = false) {
    auto trajectory = fl::TrajectoryGenerator::GenerateTrajectory(
        std::vector<fl::Pose2d>{initial, final},
        std::vector<fl::TimingConstraint<fl::Pose2dWithCurvature>*>{}, 0.0, 0.0, max_velocity,
        max_acceleration, backwards);

    fl::RamseteTracker tracker{2.0, 0.7};
    tracker.Reset(trajectory);
  }
};

TEST_F(RamseteTest, Test) {
  Run(fl::Pose2d{0.0, 0.0, fl::Rotation2d::FromDegrees(0.0)},
      fl::Pose2d{10.0, 10.0, fl::Rotation2d::FromDegrees(50.0)});
}