#pragma once

#include "fl/types/VaryInterpolatable.h"
#include "TrajectoryIterator.h"

namespace fl {
template <typename S>
class TimedEntry final : public VaryInterpolatable<TimedEntry<S>> {
 public:
  TimedEntry(const S& state, const double t, const double velocity, const double acceleration)
      : state_(state), t_(t), velocity_(velocity), acceleration_(acceleration) {}

  TimedEntry() : t_(0), velocity_(0), acceleration_(0) {}

  TimedEntry<S> Interpolate(const TimedEntry<S>& end_value, double t) const override {
    auto new_t   = this->Lerp(t_, end_value.t_, t);
    auto delta_t = new_t - this->t_;

    if (delta_t < 0.0) return end_value.Interpolate(*this, 1.0 - t);

    auto reversing = velocity_ < 0.0 || EpsilonEquals(velocity_, 0.0) && acceleration_ < 0;

    auto new_v = velocity_ + acceleration_ * delta_t;
    auto new_s = reversing ? -1.0 : 1.0 * (velocity_ * delta_t * 0.5 * acceleration_ * delta_t * delta_t);

    return TimedEntry{state_.Interpolate(end_value.state_, new_s / state_.Distance(end_value.state_)), new_t,
                      new_v, acceleration_};
  }

  double Distance(const TimedEntry<S>& other) const override { return state_.Distance(other.state_); }

  S      State() const { return state_; }
  double T() const { return t_; }
  double Velocity() const { return velocity_; }
  double Acceleration() const { return acceleration_; }

  void SetAcceleration(const double acceleration) { acceleration_ = acceleration; }

 private:
  S      state_;
  double t_;
  double velocity_;
  double acceleration_;
};

template <typename S>
class TimedIterator final : public TrajectoryIterator<double, TimedEntry<S>> {
 protected:
  double Addition(const double a, const double b) const override { return a + b; }
};

template <typename S>
class TimedTrajectory : public Trajectory<double, TimedEntry<S>> {
 public:
  TimedTrajectory(const std::vector<TimedEntry<S>>& points, const bool reversed)
      : points_(points), reversed_(reversed) {
    iterator_ = std::make_shared<TimedIterator<S>>();
    iterator_->SetTrajectory(this);
  }

  std::vector<TimedEntry<S>> Points() const override { return points_; }
  bool                       Reversed() const override { return reversed_; }

  TrajectorySamplePoint<TimedEntry<S>> Sample(const double interpolant) override {
    if (interpolant >= LastInterpolant()) {
      return TrajectorySamplePoint<TimedEntry<S>>(this->Point(points_.size() - 1));
    }
    if (interpolant <= FirstInterpolant()) {
      return TrajectorySamplePoint<TimedEntry<S>>(this->Point(0));
    }
    for (int i = 1; i < points_.size(); ++i) {
      const auto s = this->Point(i);
      if (s.state.T() >= interpolant) {
        const auto prev_s = this->Point(i - 1);
        if (EpsilonEquals(s.state.T(), prev_s.state.T())) {
          return TrajectorySamplePoint<TimedEntry<S>>(s);
        }
        return TrajectorySamplePoint<TimedEntry<S>>(
            prev_s.state.Interpolate(s.state,
                                     (interpolant - prev_s.state.T()) / (s.state.T() - prev_s.state.T())),
            i - 1, i);
      }
    }
    throw - 1;
  }

  std::shared_ptr<TrajectoryIterator<double, TimedEntry<S>>> Iterator() const override { return iterator_; }

  double        FirstInterpolant() const override { return FirstState().T(); }
  double        LastInterpolant() const override { return LastState().T(); }
  TimedEntry<S> FirstState() const override { return points_[0]; }
  TimedEntry<S> LastState() const override { return points_[points_.size() - 1]; }

 private:
  std::vector<TimedEntry<S>>                                 points_;
  bool                                                       reversed_;
  std::shared_ptr<TrajectoryIterator<double, TimedEntry<S>>> iterator_;
};

}  // namespace frc5190
