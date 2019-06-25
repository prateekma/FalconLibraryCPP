#pragma once

#include "TrajectoryIterator.h"
#include "../../types/VaryInterpolatable.h"

namespace frc5190 {
template <typename S>
class TimedEntry final : public VaryInterpolatable<TimedEntry<S>> {
 public:
  TimedEntry(const S& state, const double t, const double velocity,
             const double acceleration)
      : state_(state),
        t_(t),
        velocity_(velocity),
        acceleration_(acceleration) {}

  TimedEntry<S> Interpolate(const TimedEntry<S>& end_value, double t) override {
    auto new_t = this->Lerp(t_, end_value.t_, t);
    auto delta_t = new_t - this->t_;

    if (delta_t < 0.0) return end_value.Interpolate(*this, 1.0 - t);

    auto reversing =
        velocity_ < 0.0 || EpsilonEquals(velocity_, 0.0) && acceleration_ < 0;

    auto new_v = velocity_ + acceleration_ * delta_t;
    auto new_s = reversing ? -1.0
                           : 1.0 * (velocity_ * delta_t * 0.5 * acceleration_ *
                                    delta_t * delta_t);

    return TimedEntry{
        state_.Interpolate(end_value.state_,
                           new_s / state_.Distance(end_value.state_)),
        new_t, new_v, acceleration_};
  }

  double Distance(const TimedEntry<S>& other) const override {
    return state_.Distance(other.state_);
  }

 private:
  S state_;
  double t_;
  double velocity_;
  double acceleration_;
};

template <typename S>
class TimedIterator final : public TrajectoryIterator<double, TimedEntry<S>> {
  explicit TimedIterator(const Trajectory<double, TimedEntry<S>>& trajectory)
      : TrajectoryIterator(trajectory) {}

  double Addition(const double a, const double b) const override {
    return a + b;
  }
};

template <typename S>
class TimedTrajectory : public Trajectory<double, TimedEntry<S>> {
 public:
  TimedTrajectory(const std::vector<TimedEntry<S>>& points, const bool reversed)
      : points_(points), reversed_(reversed) {
    iterator_ = new TimedIterator<S>(this);
  }

  ~TimedTrajectory() { delete iterator_; }

  std::vector<TimedEntry<S>> Points() const override { return points_; }
  bool Reversed() const override { return reversed_; }

  TrajectoryPoint<TimedEntry<S>> Sample(const double interpolant) override {
    if (interpolant >= LastInterpolant()) {
      return TrajectorySamplePoint<TimedEntry<S>>(
          this->Point(points_.size() - 1));
    }
    if (interpolant <= FirstInterpolant()) {
      return TrajectorySamplePoint<TimedEntry<S>>(this->Point(0));
    }
    for (auto i = 1; i < points_.size(); ++i) {
      const auto s = this->Point(i);
      if (s.state.t_ >= interpolant) {
        const auto prev_s = this->Point(i - 1);
        if (EpsilonEquals(s.state.t_, prev_s.state.t_)) {
          return TrajectorySamplePoint<TimedEntry<S>>(s);
        }
        return TrajectorySamplePoint<TimedEntry<S>>(
            prev_s.state.Interpolate(s.state,
                                     (interpolant - prev_s.state.t_) /
                                         (s.state.t_ - prev_s.state.t_)),
            i - 1, i);
      }
    }
    throw - 1;
  }

  TrajectoryIterator<double, TimedEntry<S>>* Iterator() const override {
    return iterator_;
  }

  double FirstInterpolant() const override { return FirstState().t_; }
  double LastInterpolant() const override { return LastState().t_; }
  TimedEntry<S> FirstState() const override { return points_[0]; }
  TimedEntry<S> LastState() const override {
    return points_[points_.size() - 1];
  }

 private:
  std::vector<TimedEntry<S>> points_;
  bool reversed_;
  TimedIterator<S>* iterator_;
};

}  // namespace frc5190
