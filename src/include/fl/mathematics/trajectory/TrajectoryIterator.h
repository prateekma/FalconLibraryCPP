#pragma once

#include "fl/Utilities.h"
#include "Trajectory.h"

namespace fl {
template <typename U, typename S>
class TrajectoryIterator {
 public:
  TrajectoryIterator() {}
  ~TrajectoryIterator() = default;

  void SetTrajectory(Trajectory<U, S>* trajectory) {
    trajectory_ = trajectory;
    progress_   = trajectory_->FirstInterpolant();
    sample_     = trajectory_->Sample(progress_);
  }

  TrajectorySamplePoint<S> Advance(U amount) {
    progress_ =
        Clamp(Addition(progress_, amount), trajectory_->FirstInterpolant(), trajectory_->LastInterpolant());
    sample_ = trajectory_->Sample(progress_);
    return sample_;
  }

  TrajectorySamplePoint<S> Preview(U amount) {
    auto progress =
        Clamp(Addition(progress_, amount), trajectory_->FirstInterpolant(), trajectory_->LastInterpolant());
    return trajectory_->Sample(progress);
  }

  bool                     IsDone() const { return progress_ >= trajectory_->LastInterpolant(); }
  TrajectorySamplePoint<S> CurrentState() const { return sample_; }

 protected:
  virtual U Addition(U a, U b) const = 0;

 private:
  Trajectory<U, S>*        trajectory_;
  U                        progress_;
  TrajectorySamplePoint<S> sample_;
};
}  // namespace frc5190