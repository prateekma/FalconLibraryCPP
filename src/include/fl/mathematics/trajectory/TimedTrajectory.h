#pragma once

#include <utility>
#include "TrajectoryIterator.h"
#include "fl/types/VaryInterpolatable.h"

#include <units.h>

namespace fl {
template <typename S>
class TimedEntry final : public VaryInterpolatable<TimedEntry<S>> {
 public:
  TimedEntry(S state, const double t, const double velocity, const double acceleration)
      : state_(std::move(state)),
        t_(units::second_t{t}),
        velocity_(units::meters_per_second_t{velocity}),
        acceleration_(units::meters_per_second_squared_t{acceleration}) {}

  TimedEntry(S state, const units::second_t t, const units::meters_per_second_t velocity,
             const units::meters_per_second_squared_t acceleration)
      : state_(state), t_(t), velocity_(velocity), acceleration_(acceleration) {}

  TimedEntry() : t_(0), velocity_(0), acceleration_(0) {}

  TimedEntry<S> Interpolate(const TimedEntry<S>& end_value, double t) const override {
    units::second_t new_t   = this->Lerp(t_, end_value.t_, t);
    units::second_t delta_t = new_t - this->t_;

    if (delta_t < 0_s) return end_value.Interpolate(*this, 1.0 - t);

    auto reversing = velocity_ < 0_mps ||
                     EpsilonEquals(velocity_, 0_mps) && acceleration_ < 0_mps_sq;

    units::meters_per_second_t new_v = velocity_ + acceleration_ * delta_t;
    units::meter_t             new_s =
        (reversing ? -1.0 : 1.0) * (velocity_ * delta_t + 0.5 * acceleration_ * delta_t * delta_t);

    return TimedEntry{state_.Interpolate(end_value.state_,
                                         units::unit_cast<double>(new_s) / state_.Distance(end_value.state_)),
                      new_t, new_v, acceleration_};
  }

  double Distance(const TimedEntry<S>& other) const override { return state_.Distance(other.state_); }

  S                                  State() const { return state_; }
  units::second_t                    T() const { return t_; }
  units::meters_per_second_t         Velocity() const { return velocity_; }
  units::meters_per_second_squared_t Acceleration() const { return acceleration_; }

  void SetAcceleration(const double acceleration) {
    acceleration_ = units::meters_per_second_squared_t{acceleration};
  }

  void SetAcceleration(const units::meters_per_second_squared_t acceleration) {
    acceleration_ = acceleration;
  }

 private:
  S                                  state_;
  units::second_t                    t_;
  units::meters_per_second_t         velocity_;
  units::meters_per_second_squared_t acceleration_;
};

template <typename S>
class TimedIterator final : public TrajectoryIterator<units::second_t, TimedEntry<S>> {
 protected:
  units::second_t Addition(const units::second_t a, const units::second_t b) const override { return a + b; }
};

template <typename S>
class TimedTrajectory : public Trajectory<units::second_t, TimedEntry<S>> {
 public:
  TimedTrajectory(const std::vector<TimedEntry<S>>& points, const bool reversed)
      : points_(points), reversed_(reversed) {
    iterator_ = std::make_shared<TimedIterator<S>>();
    iterator_->SetTrajectory(this);
  }

  std::vector<TimedEntry<S>> Points() const override { return points_; }
  bool                       Reversed() const override { return reversed_; }

  TrajectorySamplePoint<TimedEntry<S>> Sample(const units::second_t interpolant) override {
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

  std::shared_ptr<TrajectoryIterator<units::second_t, TimedEntry<S>>> Iterator() const override {
    return iterator_;
  }

  units::second_t FirstInterpolant() const override { return FirstState().T(); }
  units::second_t LastInterpolant() const override { return LastState().T(); }
  TimedEntry<S>   FirstState() const override { return points_[0]; }
  TimedEntry<S>   LastState() const override { return points_[points_.size() - 1]; }

 private:
  std::vector<TimedEntry<S>>                                          points_;
  bool                                                                reversed_;
  std::shared_ptr<TrajectoryIterator<units::second_t, TimedEntry<S>>> iterator_;
};

}  // namespace fl
