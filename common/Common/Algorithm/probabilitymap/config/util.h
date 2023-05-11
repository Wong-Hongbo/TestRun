#ifndef CARTOGRAPHER_COMMON_MATH_H_
#define CARTOGRAPHER_COMMON_MATH_H_
#include <cmath>
#include <chrono>
#include <vector>
#include <Eigen/Core>
//#include <ceres/ceres.h>

//static double cos_lookup_table[36001];
//static double sin_lookup_table[36001];

// Clamps 'value' to be in the range ['min', 'max'].
template <typename T>
T Clamp2(const T value, const T min, const T max)
{
    if (value > max){
        return max;
    }
    if (value < min){
        return min;
    }
    return value;
}

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}


// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

// Calculates 'sqrt(x*x + y*y + z*z)'.
template <typename T>
constexpr double Dist3d(T x, T y, T z) {
  return std::sqrt(Pow2(x) + Pow2(y) + Pow2(z));
}

// Calculates 'sqrt(x*x + y*y)'.
template <typename T>
constexpr double Dist2d(T x, T y) {
  return std::sqrt(Pow2(x) + Pow2(y));
}

// Converts from degrees to radians.
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
template <typename T>
T NormalizeAngleDifference(T difference) {
  const T kPi = T(M_PI);
  while (difference > kPi) difference -= 2. * kPi;
  while (difference < -kPi) difference += 2. * kPi;
  return difference;
}

template <typename T>
T NormalizeAngleToDegree(T difference) {
  const T kPi = T(360);
  while (difference > kPi) difference -= kPi;
  while (difference < 0) difference += kPi;
  return difference;
}

//template <typename T>
//T atan2(const Eigen::Matrix<T, 2, 1>& vector) {
//  return ceres::atan2(vector.y(), vector.x());
//}


template <typename T>
inline void QuaternionProduct(const double* const z, const T* const w,
                              T* const zw) {
  zw[0] = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
  zw[1] = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
  zw[2] = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
  zw[3] = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
}

static inline int NormalizeDegreeTo36000(int degree)
{
    return (degree%36000 < 0) ?(degree%36000 + 36000):(degree%36000);
}
#endif
