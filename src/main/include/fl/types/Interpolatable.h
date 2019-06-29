#pragma once

#include "fl/Utilities.h"

namespace fl {
template <typename T>
class Interpolatable {
 public:
  virtual ~Interpolatable()                                 = default;
  virtual T Interpolate(const T& end_value, double t) const = 0;

  template<typename T>
  static constexpr T Lerp(const T& start_value, const T& end_value, const double t) {
    return start_value + (end_value - start_value) * Clamp(t, 0.0, 1.0);
  }
};
}  // namespace fl
