#ifndef UTILS_H
#define UTILS_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

const double pi = std::acos(-1);
const int NUM_ROT_ANGLES = 36001;
#define Degree2Radians(x) ((x)*pi / 180.0)

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
inline void QuaternionProduct(const double *const z, const T *const w,
                              T *const zw) {
  zw[0] = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
  zw[1] = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
  zw[2] = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
  zw[3] = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
}

static inline int NormalizeDegreeTo36000(int degree) {
  return (degree % 36000 < 0) ? (degree % 36000 + 36000) : (degree % 36000);
}

static inline int NormalizeDegreeTo360(int degree) {
  return (degree % 360 < 0) ? (degree % 360 + 360) : (degree % 360);
}

class SinuLookupTable {
 public:
  static SinuLookupTable *Instance() {
    static SinuLookupTable instance_;
    return &instance_;
  }

  double CosValue(int angle)  // 1/100 degree
  {
    int angle_mod = angle % (NUM_ROT_ANGLES - 1);
    return cos_lookup_table_[angle_mod];
  }
  double SinValue(int angle)  // 1/100 degree
  {
    int angle_mod = angle % (NUM_ROT_ANGLES - 1);
    return sin_lookup_table_[angle_mod];
  }

 private:
  SinuLookupTable(int size = NUM_ROT_ANGLES) { InitTables(size); }

  void InitTables(int size) {
    if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0) {
      cos_lookup_table_.resize(size);
      sin_lookup_table_.resize(size);
      for (int i = 0; i < size; i++) {
        double rad = DegToRad(i / 100.0);
        cos_lookup_table_[i] = std::cos(rad);
        sin_lookup_table_[i] = std::sin(rad);
      }
    }
  }

 private:
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;
};

#endif  // UTILS_H
