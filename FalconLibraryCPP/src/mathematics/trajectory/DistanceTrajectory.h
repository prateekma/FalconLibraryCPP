#pragma once

#include "TrajectoryIterator.h"

namespace frc5190 {
template <typename S>
class DistanceIterator : public TrajectoryIterator<double, S> {
 public:
  DistanceIterator(){};
  double Addition(double a, double b) const override { return a + b; }
};

template <typename S>
class DistanceTrajectory : public Trajectory<double, S> {
 public:
  explicit DistanceTrajectory(std::vector<S> points) : points_(points) {
    iterator_ = std::make_shared<DistanceIterator<S>>();

    distances_.push_back(0.0);
    for (int i = 1; i < points_.size(); ++i) {
      distances_.push_back(distances_[i - 1] +
                           points_[i - 1].Distance(points_[i]));
    }

    iterator_->SetTrajectory(this);
  }

  std::vector<S> Points() const override { return points_; }

  bool Reversed() const override { return false; }

  TrajectorySamplePoint<S> Sample(double interpolant) override {
    if (interpolant >= LastInterpolant()) {
      return TrajectorySamplePoint<S>(this->Point(points_.size() - 1));
    }
    if (interpolant <= 0.0) {
      return TrajectorySamplePoint<S>(this->Point(0));
    }

    for (int i = 1; i < distances_.size(); ++i) {
      const auto s = points_[i];
      if (distances_[i] >= interpolant) {
        const auto prev_s = points_[i - 1];
        if (EpsilonEquals(distances_[i], distances_[i - 1])) {
          return TrajectorySamplePoint<S>(s, i, i);
        }
        return TrajectorySamplePoint<S>(
            prev_s.Interpolate(s,
                               (interpolant - distances_[i - 1]) /
                                   (distances_[i] - distances_[i - 1])),
            i - 1,
            i);
      }
    }
    throw - 1;
  }

  std::shared_ptr<TrajectoryIterator<double, S>> Iterator() const override {
    return iterator_;
  }

  double FirstInterpolant() const override { return 0; }
  double LastInterpolant() const override {
    return distances_[distances_.size() - 1];
  }

  S FirstState() const override { return points_[0]; }
  S LastState() const override { return points_[points_.size() - 1]; }

 private:
  std::vector<double> distances_;
  std::vector<S> points_;
  std::shared_ptr<DistanceIterator<S>> iterator_;
};
}  // namespace frc5190
