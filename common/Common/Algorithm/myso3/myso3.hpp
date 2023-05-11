#ifndef MYSO3_HH
#define MYSO3_HH

// Extracted from "https://github.com/strasdat/Sophus"

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

using namespace Eigen;

const double SMALL_EPS = 1e-10;

class MYSO3
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MYSO3                        ();

  MYSO3                        (const MYSO3 & other);

  explicit
  MYSO3                        (const Matrix3d & _R);

  explicit
  MYSO3                        (const Quaterniond & unit_quaternion);

  MYSO3                        (double rot_x,
                              double rot_y,
                              double rot_z);
  void
  operator=                  (const MYSO3 & so3);

  MYSO3
  operator*                  (const MYSO3 & so3) const;

  void
  operator*=                 (const MYSO3 & so3);

  Vector3d
  operator*                  (const Vector3d & xyz) const;

  MYSO3
  inverse                    () const;

  Matrix3d
  matrix                     () const;

  Matrix3d
  Adj                        () const;

  Matrix3d
  generator                  (int i);

  Vector3d
  log                        () const;

  static MYSO3
  exp                        (const Vector3d & omega);

  static MYSO3
  expAndTheta                (const Vector3d & omega,
                              double * theta);
  static Vector3d
  log                        (const MYSO3 & so3);

  static Vector3d
  logAndTheta                (const MYSO3 & so3,
                              double * theta);

  static Matrix3d
  hat                        (const Vector3d & omega);

  static Vector3d
  vee                        (const Matrix3d & Omega);

  static Vector3d
  lieBracket                 (const Vector3d & omega1,
                              const Vector3d & omega2);

  static Matrix3d
  d_lieBracketab_by_d_a      (const Vector3d & b);

  void
  setQuaternion              (const Quaterniond& quaternion);

  const Quaterniond & unit_quaternion() const
  {
    return unit_quaternion_;
  }

  static const int DoF = 3;


protected:
  Quaterniond unit_quaternion_;

};


inline std::ostream& operator <<(std::ostream & out_str,
                                 const MYSO3 & so3)
{

  out_str << so3.log().transpose() << std::endl;
  return out_str;
}


#endif
