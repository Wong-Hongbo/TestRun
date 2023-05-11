/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     planeDetect.cc
* @brief
* @author       Yu Hui Liang
* @date         2020-09-21 23:34:45
*/
#include "Common/Algorithm/planeDetect.hh"

PLANEDETECT::PLANEDETECT(MAPDATAFRAME* MapDataFrame):mMapDataFrame(MapDataFrame)
{
    ransac_distance_thresh = 1.0;
}

PLANEDETECT::~PLANEDETECT()
{

}

void PLANEDETECT::RegionGrowing(pcl::PointXYZI pt, double radius)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree1(new pcl::search::KdTree<pcl::PointXYZI>);

    std::vector<int> indices;
    std::vector<float> squared_distances;
    kdtree1->setInputCloud(mMapDataFrame->cloud_keyposes_3d_ptr);
    kdtree1->radiusSearch(pt, radius, indices, squared_distances);

    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_points(new pcl::PointCloud<pcl::PointXYZI>());

    for(int i=0;i< indices.size();i++)
    {
        *accumulated_points += *GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[i].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[i]);
    }

    pcl::PointCloud<pcl::Normal>::Ptr accumulated_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setInputCloud(accumulated_points);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_estimation_radius);
    ne.compute(*accumulated_normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize(min_cluster_size);
    reg.setMaxClusterSize(max_cluster_size);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointXYZI>());
    kdtree2->setInputCloud(accumulated_points);
    reg.setSearchMethod(kdtree2);

    reg.setNumberOfNeighbours(num_neighbors);
    reg.setInputCloud(accumulated_points);
    reg.setInputNormals(accumulated_normals);
    reg.setSmoothnessThreshold(smoothness_threshold / 180.0f * M_PI);
    reg.setCurvatureThreshold(curvature_threshold);

    pcl::PointIndices::Ptr cluster(new pcl::PointIndices());
    reg.getSegmentFromPoint(0, *cluster);

    candidate_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    normals.reset(new pcl::PointCloud<pcl::Normal>());

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(accumulated_points);
    extract.setIndices(cluster);
    extract.setNegative(false);
    extract.filter(*candidate_cloud);

    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setInputCloud(accumulated_normals);
    extract_normals.setIndices(cluster);
    extract_normals.setNegative(false);
    extract_normals.filter(*normals);
}


void PLANEDETECT::DetectPlane()
{
    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(candidate_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
    ransac.setDistanceThreshold(ransac_distance_thresh);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    ransac.getInliers(inliers->indices);
//    PlaneDetectionResult::Ptr result(new PlaneDetectionResult());
    ransac.getModelCoefficients(result_coeffs);

//    for (const auto& candidate : rg_result->candidates) {
//      Eigen::Vector4f coeffs = result_coeffs;
//      Eigen::Vector4f local_coeffs;

//      Eigen::Isometry3f trans = candidate->node->estimate().inverse().cast<float>();
//      local_coeffs.head<3>() = trans.linear() * coeffs.head<3>();
//      local_coeffs[3] = coeffs[3] - trans.translation().dot(local_coeffs.head<3>());

//      auto inliers = DetectPlaneWithCoeffs(candidate_cloud, local_coeffs);
//      if (inliers == nullptr || inliers->size() < min_plane_supports) {
//        continue;
//      }
////      result->candidates.push_back(candidate);
////      result->candidate_inliers.push_back(inliers);
////      result->candidate_inlier_buffers.push_back(std::make_shared<glk::PointCloudBuffer>(inliers));
////      result->candidate_local_coeffs.push_back(local_coeffs);
//    }
//    return result;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr PLANEDETECT::DetectPlaneWithCoeffs(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, Eigen::Vector4f coeffs)
{
    pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(in_cloud));
    pcl::PointIndices::Ptr init_indices(new pcl::PointIndices);
    model_p->selectWithinDistance(coeffs, ransac_distance_thresh, init_indices->indices);

    if (init_indices->indices.size() < 10) {
      return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr init_inliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices(init_indices);
    extract.setNegative(false);
    extract.filter(*init_inliers);

    model_p.reset(new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(init_inliers));
    pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
    ransac.setDistanceThreshold(ransac_distance_thresh);
    ransac.computeModel();

    Eigen::VectorXf coeffs_;
    ransac.getModelCoefficients(coeffs_);

    if (coeffs_.head<3>().dot(coeffs.head<3>()) < 0.0f) {
      coeffs_ *= -1.0f;
    }
    coeffs = coeffs_;

    pcl::PointIndices::Ptr indices(new pcl::PointIndices());
    ransac.getInliers(indices->indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZI>());
    extract.setInputCloud(init_inliers);
    extract.setIndices(indices);
    extract.filter(*inliers);

    return inliers;
}
