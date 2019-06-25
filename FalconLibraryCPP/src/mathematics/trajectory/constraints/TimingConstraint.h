#pragma once

#include <limits>

namespace frc5190 {

struct MinMaxAcceleration {
  double min_acceleration;
  double max_acceleration;

  bool IsValid() const { return min_acceleration < max_acceleration; }
};

template <typename S>
class TimingConstraint {
 public:
  virtual ~TimingConstraint() = default;
  static constexpr MinMaxAcceleration kNoLimits =
      MinMaxAcceleration{std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::max()};

  virtual double MaxVelocity(const S& state) const = 0;
  virtual MinMaxAcceleration MinMaxAcceleration(const S& state,
                                                double velocity) const = 0;
};
}  // namespace frc5190
