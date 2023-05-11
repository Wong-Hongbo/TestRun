/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     m2dp_yhl.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-09-17 17:45:22
*/
#ifndef DESCRIPTOR_M2DP_H_
#define DESCRIPTOR_M2DP_H_

// third party
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

using Eigen_Descriptor = Eigen::VectorXf;
struct PolarCoordinate {
  double angle, length;
};

inline double getLength(pcl::PointXYZI a) {
  return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}
class M2DP {
public:
    M2DP(double r = 0.1, double max_distance = 100., int32_t t = 16,
         int32_t p = 4, int32_t q = 16);
    ~M2DP() = default;

    Eigen_Descriptor getFinalDescriptor();

//    bool setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr source);
    bool setInputSourceCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr source);
    bool setInputTargetCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr source);

    void preProcess(pcl::PointCloud<pcl::PointXYZI>::Ptr source);

    Eigen::VectorXi singleViewProcess(double theta, double phi);

    pcl::PointCloud<pcl::PointXYZI>::Ptr inner_cloud_;
    Eigen::Vector3f mean_;

    // parameters in paper
    int32_t l_, t_;
    int32_t p_, q_;
    double r_;
    double max_distance_;

    Eigen::MatrixXf A_;

    Eigen_Descriptor descriptor1_;
    Eigen_Descriptor descriptor2_;

    double matchTwoM2dpDescriptors() ;
};

#endif  // DESCRIPTOR_M2DP_H_
