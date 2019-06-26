#include <FalconLibrary.h>
#include <benchmark/benchmark.h>

static void BM_ArcParam(benchmark::State& state) {
  const frc5190::Pose2d initial{0.0, 0.0, frc5190::Rotation2d::FromDegrees(0.0)};
  const frc5190::Pose2d final{10.0, 10.0, frc5190::Rotation2d::FromDegrees(50.0)};
  const auto            spline = std::make_shared<frc5190::ParametricQuinticHermiteSpline>(initial, final);

  for (auto _ : state) {
    frc5190::SplineGenerator::ParameterizeSpline(spline, 2.0, 0.05, 0.1);
  }
}

BENCHMARK(BM_ArcParam)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);