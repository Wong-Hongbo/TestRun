#ifndef MATH_H
#define MATH_H
#include <cmath>
#include <vector>

#include <eigen3/Eigen/Core>
#include "port.h"
//#include "ceres/ceres.h"

// Clamps 'value' to be in the range ['min', 'max'].
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power_raw(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2_2(T a) {
  return Power_raw(a, 2);
}

// Converts from degrees to radians.
constexpr double DegToRad2(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg2(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
template <typename T>
T NormalizeAngleDifference2(T difference)
{
  const T kPi = T(M_PI);
  while (difference > kPi) difference -= 2. * kPi;
  while (difference < -kPi) difference += 2. * kPi;
  return difference;
}

// template <typename T>
// T atan2(const Eigen::Matrix<T, 2, 1>& vector) 
// {
//   return ceres::atan2(vector.y(), vector.x());
// }

template <typename T>
inline void QuaternionProduct2(const double* const z, const T* const w,T* const zw) 
{
  zw[0] = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
  zw[1] = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
  zw[2] = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
  zw[3] = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
}
#endif // MATH_H
