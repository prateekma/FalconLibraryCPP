#include <FalconLibrary.h>
#include <benchmark/benchmark.h>

static void BM_TrajectoryGeneration(benchmark::State& state) {
  const frc5190::Pose2d initial{0.0, 0.0, frc5190::Rotation2d::FromDegrees(0.0)};
  const frc5190::Pose2d final{10.0, 10.0, frc5190::Rotation2d::FromDegrees(50.0)};

  for (auto _ : state) {
    frc5190::TrajectoryGenerator::GenerateTrajectory(
        std::vector<frc5190::Pose2d>{initial, final},
        std::vector<frc5190::TimingConstraint<frc5190::Pose2dWithCurvature>*>{}, 0.0, 0.0, 3., 2., false);
  }
}

BENCHMARK(BM_TrajectoryGeneration)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);