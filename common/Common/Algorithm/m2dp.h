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

//namespace descriptor {

// refer to the paper
// "M2DP: A Novel 3D Point Cloud Descriptor and Its Application in Loop
// Closure Detection"
template <typename PointType>
class M2dp {
 public:
  typedef pcl::PointCloud<PointType> PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  using Descriptor = Eigen::VectorXf;

  M2dp(double r = 0.1, double max_distance = 100., int32_t t = 16,
       int32_t p = 4, int32_t q = 16);
  ~M2dp() = default;

  bool setInputCloud(const PointCloudSourcePtr& source);

  Descriptor getFinalDescriptor();

 private:
  // part III.B in paper
  void preProcess(const PointCloudSourcePtr& source);
  // part III.C in paper
  Eigen::VectorXi singleViewProcess(double theta, double phi);

  PointCloudSourcePtr inner_cloud_;
  Eigen::Vector3f mean_;

  // parameters in paper
  int32_t l_, t_;
  int32_t p_, q_;
  double r_;
  double max_distance_;

  Eigen::MatrixXf A_;
  Descriptor descriptor_;
};

template <typename PointType>
inline typename M2dp<PointType>::Descriptor
M2dp<PointType>::getFinalDescriptor() {
  return descriptor_;
}

// return the match score ( 0, 1 )
template <typename PointType>
double matchTwoM2dpDescriptors(const typename M2dp<PointType>::Descriptor& P,
                               const typename M2dp<PointType>::Descriptor& Q);

//}  // namespace descriptor

#endif  // DESCRIPTOR_M2DP_H_
