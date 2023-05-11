/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     planeDetect.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-09-22 09:22:55
*/
#ifndef PLANE_DETECT_HH_
#define PLANE_DETECT_HH_
#include <Common/MapDataFrame/MapDataStruct.hh>
#include <Common/Commonfig.hh>
#include <Common/ParamServer.hh>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/region_growing.hpp>

class PLANEDETECT
{
public:
    PLANEDETECT(MAPDATAFRAME* MapDataFrame);
    ~PLANEDETECT();

    void RegionGrowing(pcl::PointXYZI pt,double radius);
    std::vector<int> neighbor_indices;
    void DetectPlane();

    pcl::PointCloud<pcl::PointXYZI>::Ptr DetectPlaneWithCoeffs(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Eigen::Vector4f coeffs);

    pcl::PointCloud<pcl::PointXYZI>::Ptr candidate_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;

    Eigen::VectorXf result_coeffs;

    float initial_neighbor_search_radius;
    float normal_estimation_radius;
    int min_cluster_size;
    int max_cluster_size;
    int num_neighbors;
    float smoothness_threshold;
    float curvature_threshold;

    float ransac_distance_thresh;
    int min_plane_supports;

protected:
    MAPDATAFRAME* mMapDataFrame;

};

#endif
