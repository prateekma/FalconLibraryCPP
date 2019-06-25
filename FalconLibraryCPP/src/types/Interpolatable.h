#pragma once

#include "Utilities.h"

namespace frc5190 {
template <typename T>
class Interpolatable {
 public:
  virtual ~Interpolatable() = default;
  virtual T Interpolate(const T& end_value, double t) const = 0;

  static constexpr double Lerp(const double start_value, const double end_value,
                               const double t) {
    return start_value + (end_value - start_value) * Clamp(t, 0.0, 1.0);
  }
};
}  // namespace frc5190
