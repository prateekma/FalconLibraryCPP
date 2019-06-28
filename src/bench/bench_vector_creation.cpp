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

  ConstrainedPose() {}

  ConstrainedPose(const S& state, double distance, double max_velocity, double min_acceleration,
                  double max_acceleration) {
    this->state            = state;
    this->distance         = distance;
    this->max_velocity     = max_velocity;
    this->min_acceleration = min_acceleration;
    this->max_acceleration = max_acceleration;
  }

  ConstrainedPose(ConstrainedPose& other) = default;
  ConstrainedPose& operator=(ConstrainedPose& other) {
    //std::cout << "Copy operator called";
    this->state            = other.state;
    this->distance         = other.distance;
    this->max_velocity     = other.max_velocity;
    this->min_acceleration = other.min_acceleration;
    this->max_acceleration = other.max_acceleration;
    return *this;
  }

  ConstrainedPose& operator=(ConstrainedPose&& other) = default;
};

static void BM_VectorReserveAndPushBack(benchmark::State& state) {
  for (auto _ : state) {
    std::vector<ConstrainedPose<fl::Pose2dWithCurvature>> vector;
    vector.reserve(kVectorSize);

    for (int i = 0; i < kVectorSize; i++) {
      vector.push_back({fl::Pose2dWithCurvature{}, 1.0 * i, 1.0 * i, 1.0 * i, 1.0 * i});
    }
  }
}

static void BM_VectorReserveAndEmplaceBack(benchmark::State& state) {
  for (auto _ : state) {
    std::vector<ConstrainedPose<fl::Pose2dWithCurvature>> vector;
    vector.reserve(kVectorSize);

    for (int i = 0; i < kVectorSize; i++) {
      vector.emplace_back(fl::Pose2dWithCurvature{}, 1.0 * i, 1.0 * i, 1.0 * i, 1.0 * i);
    }
  }
}

static void BM_VectorConstructorAndAssignIndices(benchmark::State& state) {
  for (auto _ : state) {
    std::vector<ConstrainedPose<fl::Pose2dWithCurvature>> vector{kVectorSize};
    for (int i = 0; i < kVectorSize; i++) {
      vector[i] = ConstrainedPose<fl::Pose2dWithCurvature>(fl::Pose2dWithCurvature{}, 1.0 * i, 1.0 * i,
                                                           1.0 * i, 1.0 * i);
    }
  }
}

BENCHMARK(BM_VectorReserveAndEmplaceBack)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);
BENCHMARK(BM_VectorReserveAndPushBack)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);
BENCHMARK(BM_VectorConstructorAndAssignIndices)->Arg(100)->Complexity()->Unit(benchmark::kMillisecond);