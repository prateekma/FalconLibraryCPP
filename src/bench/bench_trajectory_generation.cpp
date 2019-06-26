#include <FalconLibrary.h>
#include <benchmark/benchmark.h>

#include <vector>

static void BM_TrajectoryGeneration(benchmark::State& state) {
  const fl::Pose2d initial{0.0, 0.0, fl::Rotation2d::FromDegrees(0.0)};
  const fl::Pose2d final{10.0, 10.0, fl::Rotation2d::FromDegrees(45.0)};

  for (auto _ : state) {
    fl::TrajectoryGenerator::GenerateTrajectory(std::vector<fl::Pose2d>{initial, final},
                                                std::vector<fl::TimingConstraint<fl::Pose2dWithCurvature>*>{},
                                                0.0, 0.0, 3., 2., false);
  }
}

BENCHMARK(BM_TrajectoryGeneration)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);