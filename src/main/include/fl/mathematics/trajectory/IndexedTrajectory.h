#pragma once

#include "Trajectory.h"
#include "TrajectoryIterator.h"

#include <cmath>
#include <limits>

namespace fl {

constexpr double kLowestDouble = std::numeric_limits<double>::lowest();

template <typename S>
class IndexedIterator : public TrajectoryIterator<double, S> {
 public:
  IndexedIterator() {}

 protected:
  double Addition(const double a, const double b) const override { return a + b; }
};

template <typename S>
class IndexedTrajectory : public Trajectory<double, S> {
 public:
  explicit IndexedTrajectory(const std::vector<S>& points) : points_(points) {
    iterator_ = std::make_shared<IndexedIterator<S>>();
    iterator_->SetTrajectory(this);
  }

  std::vector<S> Points() const override { return points_; }

  bool Reversed() const override { return false; }

  TrajectorySamplePoint<S> Sample(double interpolant) override {
    if (points_.empty()) throw - 1;
    if (interpolant <= 0.0) {
      return TrajectorySamplePoint<S>(this->Point(0.0));
    }
    if (interpolant >= points_.size() - 1) {
      return TrajectorySamplePoint<S>(this->Point(points_.size() - 1));
    }

    const auto index   = static_cast<int>(std::floor(interpolant));
    const auto percent = interpolant - index;

    if (percent <= kLowestDouble) {
      return TrajectorySamplePoint<S>(this->Point(index));
    }
    if (percent >= 1 - kLowestDouble) {
      return TrajectorySamplePoint<S>(this->Point(index + 1));
    }
    return TrajectorySamplePoint<S>(points_[index].Interpolate(points_[index], percent), index, index + 1);
  }

  double FirstInterpolant() const override { return 0.0; }
  double LastInterpolant() const override { return std::max(0.0, points_.size() - 1.0); }

  S FirstState() const override { return points_[0]; }
  S LastState() const override { return points_[points_.size() - 1]; }

  std::shared_ptr<TrajectoryIterator<double, S>> Iterator() const override { return iterator_; }

 private:
  std::vector<S>                                 points_;
  std::shared_ptr<TrajectoryIterator<double, S>> iterator_;
};

}  // namespace fl
