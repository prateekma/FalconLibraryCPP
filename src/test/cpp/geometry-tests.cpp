#include <gtest/gtest.h>
#include "FalconLibrary.h"

constexpr double kTestEpsilon = 1E-9;

TEST(TestRotation2d, TestRotation2d) {
  auto rot = fl::Rotation2d();
  EXPECT_EQ(1.0, rot.Cos());
  EXPECT_EQ(0.0, rot.Sin());
  EXPECT_EQ(0.0, rot.Tan());
  EXPECT_EQ(0.0, rot.Radians());
  EXPECT_EQ(0.0, rot.Degrees());

  rot = fl::Rotation2d(1, 1, true);
  EXPECT_NEAR(std::sqrt(2) / 2, rot.Cos(), kTestEpsilon);
  EXPECT_NEAR(std::sqrt(2) / 2, rot.Sin(), kTestEpsilon);
  EXPECT_NEAR(1.0, rot.Tan(), kTestEpsilon);
  EXPECT_NEAR(45.0, rot.Degrees(), kTestEpsilon);
  EXPECT_NEAR(fl::kPi / 4, rot.Radians(), kTestEpsilon);

  auto twist = fl::Twist2d();
}