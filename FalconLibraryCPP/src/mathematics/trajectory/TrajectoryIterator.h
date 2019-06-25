#pragma once

#include "Trajectory.h"
#include "../../Utilities.h"

namespace frc5190 {
template <typename U, typename S>
class TrajectoryIterator {

  ~TrajectoryIterator() = default;

  explicit TrajectoryIterator(Trajectory<U, S>* trajectory)
      : trajectory_(trajectory) {}

  virtual U Addition(U a, U b) const = 0;

  TrajectorySamplePoint<S> Advance(U amount) {
    progress_ =
        Clamp(Addition(progress_, amount), trajectory_->FirstInterpolant(),
              trajectory_->LastInterpolant());
    sample_ = trajectory_->Sample(progress_);
    return sample_;
  }

  TrajectorySamplePoint<S> Preview(U amount) {
    auto progress =
        Clamp(Addition(progress_, amount), trajectory_->FirstInterpolant(),
              trajectory_->LastInterpolant());
    return trajectory_->Sample(progress);
  }

  bool IsDone() const { return progress_ >= trajectory_->LastInterpolant(); }
  TrajectoryPoint<S> CurrentState() const { return sample_; }

 private:
  Trajectory<U, S>* trajectory_;
  auto progress_ = trajectory_ -> FirstInterpolant();
  auto sample_ = trajectory_ -> Sample(progress_);
};
}  // namespace frc5190