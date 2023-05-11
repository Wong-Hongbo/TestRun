/*
 * @Description: 类型定义
 * @Author: zhong haixing
 * @Date: 2020-12-02 10:28:04
 * @Email: zhonghaixing@xingshentech.com
 * @FilePath: /common/type.h
 */
#ifndef TYPE_H
#define TYPE_H

#include <iostream>
#include <map>
#include <unordered_map>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <limits>
#include <common/color.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "common/math1.h"

static const uint64_t UNIVERSAL_CONSTANT = 2642244;
// static const uint64_t UNIVERSAL_CONSTANT = 327680;
static const uint64_t max_uint32_t = std::numeric_limits<uint32_t>::max();
static const uint64_t min_uint32_t = std::numeric_limits<uint32_t>::min();
static const float max_float = std::numeric_limits<float>::max();
static const float min_float = std::numeric_limits<float>::min();

static const int8_t log_odds_min = -120;
static const int8_t log_odds_max = 120;

static const int8_t free_threshold = -8;
static const int8_t occ_threshold = 2;
static const uint16_t max_sqdist = 31 *31;
// static const uint16_t max_sqdist = 7 *7;
#define limit(x, min, max)                     \
    {                                          \
        x = x < min ? min : x > max ? max : x; \
    }
typedef Eigen::Matrix<uint32_t, 2, 1> Vector2ui;
typedef Eigen::Matrix<float, 2, 1> Vector2f;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<int64_t, 2, 1> Vector2l;

typedef std::vector<Eigen::Vector2f> PointClouds;
typedef std::vector<uint8_t> Intensity;

typedef pcl::PointXYZ PCLPOINT;
typedef pcl::PointCloud<PCLPOINT> PCLPoints;
typedef pcl::PointCloud<PCLPOINT>::Ptr PCLPoints_ptr;

//struct PointXYZIRPYT
//{
//    PCL_ADD_POINT4D
//    PCL_ADD_INTENSITY;
//    float roll;
//    float pitch;
//    float yaw;
//    double time;
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//} EIGEN_ALIGN16;
//POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

//using PointType6D = PointXYZIRPYT;

//点云数据结构
struct Point_cloud2d
{
    PointClouds points;
    Eigen::Vector3f robot_pose;
};

struct Keyframe
{
    double time;
    int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
};

struct LoopPose
{
    double time;
    int index0 = 0;
    int index1 = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
};

struct P_map_cell
{
    int visit = -1;
    int hit_num = 0;
};

//ceres用到的约束
struct Constraint {

  struct Pose
  {
      Eigen::Matrix4f pose;
      double translation_weight;
      double rotation_weight;
  };


  int index0 = 0;
  int index1 = 0;

  // Pose of the node 'j' relative to submap 'i'.
  Pose pose;

  // Differentiates between intra-submap (where node 'j' was inserted into
  // submap 'i') and inter-submap constraints (where node 'j' was not inserted
  // into submap 'i').
  enum Tag { INTRA_SUBMAP, INTER_SUBMAP, NODE, MANUAL} tag;
};

template <typename T>
Eigen::Matrix4f array_to_matrix(std::array<T, 3> pose_2d){
    Eigen::Matrix4f pose_3d = Eigen::Matrix4f::Identity();
    pose_3d(0, 3) = pose_2d[0];
    pose_3d(1, 3) = pose_2d[1];
    pose_3d.block<3, 3>(0, 0) = Eigen::AngleAxisf(pose_2d[2], Eigen::Vector3f(0, 0, 1)).matrix();
    return pose_3d;
}
std::array<float, 3> matrix_to_array(const Eigen::Matrix4f& pose_3d);
std::array<double, 3> matrix_to_arrayd(const Eigen::Matrix4f& pose_3d);

//从四元数中获取yaw
float getYawEulerAngle(const Eigen::Quaternionf& q);

Eigen::Matrix4f array_to_matrix_3d(std::array<float, 6> pose_3d);
std::array<float, 6> matrix_to_array_3d(Eigen::Matrix4f pose_3d);

Point_cloud2d PclToPointcloud2d(pcl::PointCloud<pcl::PointXYZI> cloud_, Eigen::Matrix4f& scan_pose_);
Point_cloud2d PclToPointcloud2d(PCLPoints& pcl_cloud);
PCLPoints Pointcloud2dToPcl(Point_cloud2d& cloud_2d);


template <typename FloatType>
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector& translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid2 Rotation(const double rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Rotation(const Rotation2D& rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector& vector) {
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2<FloatType> Identity() { return Rigid2<FloatType>(); }

  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }

  Rotation2D rotation() const { return rotation_; }

  double normalized_angle() const {
    return NormalizeAngleDifference2(rotation().angle());
  }

  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

  std::string DebugString() const {
    std::string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append("], r: [");
    out.append(std::to_string(rotation().angle()));
    out.append("] }");
    return out;
  }

 private:
  Vector translation_;
  Rotation2D rotation_;
};

template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const Rigid2<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Pose2Dd = Rigid2<double>;
using Pose2Df = Rigid2<float>;

struct Keyframe_Scan
{
    Keyframe_Scan(){}
    Keyframe_Scan(PCLPoints_ptr scan_, Pose2Df pose_):scan(scan_), pose(pose_){}
    PCLPoints_ptr scan = nullptr;
    Pose2Df pose;
};

#endif // TYPE_H
