#include <FalconLibrary.h>
#include <benchmark/benchmark.h>

static void BM_ArcParam(benchmark::State& state) {
  const fl::Pose2d initial{0.0, 0.0, fl::Rotation2d::FromDegrees(0.0)};
  const fl::Pose2d final{10.0, 10.0, fl::Rotation2d::FromDegrees(45.0)};
  const auto       spline = std::make_shared<fl::ParametricQuinticHermiteSpline>(initial, final);

  for (auto _ : state) {
    fl::SplineGenerator::ParameterizeSpline(spline, 2.0, 0.05, 0.1);
  }
}

BENCHMARK(BM_ArcParam)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);