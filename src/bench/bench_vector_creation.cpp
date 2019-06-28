#include <benchmark/benchmark.h>
#include <vector>
#include "FalconLibrary.h"

static constexpr int kVectorSize = 500;

template <typename S>
struct ConstrainedPose {
  S      state;
  double distance         = 0.0;
  double max_velocity     = 0.0;
  double min_acceleration = 0.0;
  double max_acceleration = 0.0;
};

static void BM_VectorReserveAndPushBack(benchmark::State& state) {
  for (auto _ : state) {
    std::vector<ConstrainedPose<fl::Pose2dWithCurvature>> vector;
    vector.reserve(kVectorSize);

    for (int i = 0; i < kVectorSize; i++) {
      vector.push_back({fl::Pose2dWithCurvature{}, 1.0 * i, 1.0 * i, 1.0 * i});
    }
  }
}

static void BM_VectorConstructor(benchmark::State& state) {
  for (auto _ : state) {
    std::vector<ConstrainedPose<fl::Pose2dWithCurvature>> vector{kVectorSize};
    for (int i = 0; i < kVectorSize; i++) {
      vector[i] = {fl::Pose2dWithCurvature{}, 1.0 * i, 1.0 * i, 1.0 * i};
    }
  }
}

BENCHMARK(BM_VectorReserveAndPushBack)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);
BENCHMARK(BM_VectorConstructor)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);