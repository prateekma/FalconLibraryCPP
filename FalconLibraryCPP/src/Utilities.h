#pragma once

#include <algorithm>

namespace frc5190 {

constexpr double kEpsilon = 1E-9;
constexpr double kPi = 3.14159265358979323846;

template <typename T>
T Clamp(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

template <typename T>
constexpr T Rad2Deg(const T& rad) {
  return rad * 180.0 / kPi;
}

template <typename T>
constexpr T Deg2Rad(const T& deg) {
  return deg * kPi / 180.0;
}

template <typename T>
bool EpsilonEquals(const T& a, const T& b) {
  return std::abs(a - b) < kEpsilon;
}

}  // namespace frc5190