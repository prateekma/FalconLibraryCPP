#pragma once

#include "Interpolatable.h"

namespace frc5190 {
template <typename T>
class VaryInterpolatable : public Interpolatable<T> {
 public:
  virtual double Distance(const T& other) const = 0;
};
}  // namespace frc5190
