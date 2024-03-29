#pragma once

#include <memory>
#include <utility>
#include <vector>

namespace fl {

template <typename S>
struct TrajectoryPoint {
  int index;
  S   state;
};

template <typename S>
struct TrajectorySamplePoint {
  S   state;
  int index_floor;
  int index_ceil;

  explicit TrajectorySamplePoint(TrajectoryPoint<S> point)
      : state(point.state), index_floor(point.index), index_ceil(point.index) {}

  TrajectorySamplePoint(S state, int index_floor, int index_ceil)
      : state(std::move(state)), index_floor(index_floor), index_ceil(index_ceil) {}

  TrajectorySamplePoint() : index_floor(0), index_ceil(0) {}
};

template <typename U, typename S>
class TrajectoryIterator;

template <typename U, typename S>
class Trajectory {
 public:
  virtual std::vector<S> Points() const   = 0;
  virtual bool           Reversed() const = 0;

  TrajectoryPoint<S> Point(int index) const { return TrajectoryPoint<S>{index, Points()[index]}; }

  virtual TrajectorySamplePoint<S> Sample(U interpolant) = 0;

  virtual std::shared_ptr<TrajectoryIterator<U, S>> Iterator() const = 0;

  virtual U FirstInterpolant() const = 0;
  virtual U LastInterpolant() const  = 0;

  virtual S FirstState() const = 0;
  virtual S LastState() const  = 0;
};

}  // namespace fl