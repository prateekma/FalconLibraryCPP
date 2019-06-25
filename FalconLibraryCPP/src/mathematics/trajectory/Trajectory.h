#pragma once

#include <vector>

namespace frc5190 {

template <typename S>
struct TrajectoryPoint {
  int index;
  S state;
};

template <typename S>
struct TrajectorySamplePoint {
  S state;
  int index_floor;
  int index_ceil;

  explicit TrajectorySamplePoint(TrajectoryPoint<S> point)
      : state(point.state), index_floor(point.index), index_ceil(point.index) {}
};

template <typename U, typename S>
class TrajectoryIterator;

template <typename U, typename S>
class Trajectory {
 public:
  virtual std::vector<S> Points() const = 0;
  virtual bool Reversed() const = 0;

  TrajectoryPoint<S> Point(int index) const {
    return TrajectoryPoint<S>(index, Points()[index]);
  }

  virtual TrajectoryPoint<S> Sample(U interpolant) = 0;

  virtual TrajectoryIterator<U, S>* Iterator() const = 0;

  virtual U FirstInterpolant() const = 0;
  virtual U LastInterpolant() const = 0;

  virtual S FirstState() const = 0;
  virtual S LastState() const = 0;
};

}  // namespace frc5190