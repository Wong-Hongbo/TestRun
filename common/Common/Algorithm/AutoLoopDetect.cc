/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     AutoLoopDetect.cc
* @brief
* @author       Yu Hui Liang
* @date         2020-12-30
*/
/**************************************************************************************************/
#include "Common/Algorithm/AutoLoopDetect.hh"
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d_omp.h>
#include "Common/Algorithm/match_core/ndt/ndt_omp.h"
#include "Common/Algorithm/m2dp.h"
#include "pcl/registration/icp.h"
AUTOMATCHING::AUTOMATCHING(MAPDATAFRAME *MapDataFrame):mMapDataFrame(MapDataFrame),
    fpfh_normal_estimation_radius(0.05f),
    fpfh_search_radius(0.1f),
    fpfh_max_iterations(20000),
    fpfh_num_samples(3),
    fpfh_correspondence_randomness(5),
    fpfh_similarity_threshold(0.85f),
    fpfh_max_correspondence_distance(0.30f),
    fpfh_inlier_fraction(0.25f)
{
}

AUTOMATCHING::~AUTOMATCHING()
{
}

Eigen::Matrix4f AUTOMATCHING::AutoMatching(int start_id,int end_id)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr start_keyframe_cloud(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr end_keyframe_cloud(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<FeatureT>::Ptr start_keyframe_features(new pcl::PointCloud<FeatureT>());
    pcl::PointCloud<FeatureT>::Ptr end_keyframe_features(new pcl::PointCloud<FeatureT>());

    mMapDataFrame->start_keyframe_ptr->clear();
    mMapDataFrame->end_keyframe_ptr->clear();
    mMapDataFrame->start_keyframe_ptr = GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[start_id].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[start_id]);
    mMapDataFrame->end_keyframe_ptr = GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[end_id].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[end_id]);

    pcl::copyPointCloud(*mMapDataFrame->start_keyframe_ptr, *start_keyframe_cloud);
    pcl::copyPointCloud(*mMapDataFrame->end_keyframe_ptr, *end_keyframe_cloud);

    pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
    nest.setRadiusSearch(fpfh_normal_estimation_radius);
    nest.setInputCloud(start_keyframe_cloud);
    nest.compute(*start_keyframe_cloud);
    nest.setInputCloud(end_keyframe_cloud);
    nest.compute(*end_keyframe_cloud);

    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
    fest.setRadiusSearch(fpfh_search_radius);
    fest.setInputCloud(start_keyframe_cloud);
    fest.setInputNormals(start_keyframe_cloud);
    fest.compute(*start_keyframe_features);
    fest.setInputCloud(end_keyframe_cloud);
    fest.setInputNormals(end_keyframe_cloud);
    fest.compute(*end_keyframe_features);


    pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
    align.setInputSource(end_keyframe_cloud);
    align.setSourceFeatures(end_keyframe_features);
    align.setInputTarget(start_keyframe_cloud);
    align.setTargetFeatures(start_keyframe_features);

    align.setMaximumIterations(fpfh_max_iterations);
    align.setNumberOfSamples(fpfh_num_samples);
    align.setCorrespondenceRandomness(fpfh_correspondence_randomness);
    align.setSimilarityThreshold(fpfh_similarity_threshold);
    align.setMaxCorrespondenceDistance(fpfh_max_iterations);
    align.setInlierFraction(fpfh_inlier_fraction);

    pcl::PointCloud<pcl::PointNormal>::Ptr aligned(new pcl::PointCloud<pcl::PointNormal>());
    align.align(*aligned);

    Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
    relative.matrix() = align.getFinalTransformation().cast<double>();
    Eigen::Matrix4f matrix = align.getFinalTransformation();
    UpdateFitnessScore(matrix);
    return matrix;
}

Eigen::Matrix4f AUTOMATCHING::ScanMatching(int start_id,int end_id,PointType6D trans_pose)
{
    mMapDataFrame->start_keyframe_ptr->clear();
    mMapDataFrame->end_keyframe_ptr->clear();
    mMapDataFrame->start_keyframe_ptr = GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[start_id].makeShared(),trans_pose);
//    mMapDataFrame->end_keyframe_ptr = GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[end_id].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[end_id]);

    *mMapDataFrame->end_keyframe_ptr += *mMapDataFrame->near_history_keyframes_ptr;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_start_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setLeafSize(0.2,0.2,0.2);
    filter.setInputCloud(mMapDataFrame->start_keyframe_ptr);
    filter.filter(*filter_start_point);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_end_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> filter2;
    filter2.setLeafSize(0.4,0.4,0.4);
    filter2.setInputCloud(mMapDataFrame->end_keyframe_ptr);
    filter2.filter(*filter_end_point);

    pclomp::NormalDistributionsTransform<PointType3D, PointType3D>::Ptr NdtAligner;
    NdtAligner.reset(new pclomp::NormalDistributionsTransform<PointType3D, PointType3D>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
    NdtAligner->setMaximumIterations(64);
    NdtAligner->setNumThreads(7);
    NdtAligner->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    NdtAligner->setInputTarget(filter_end_point);
    NdtAligner->setTransformationEpsilon(0.000001);
    NdtAligner->setStepSize(0.1);
    NdtAligner->setResolution(2.0);
    NdtAligner->setInputSource(filter_start_point);
    NdtAligner->align(*aligned);
    Eigen::Matrix4f align_matrix = NdtAligner->getFinalTransformation();
    pcl::PointCloud<pcl::PointXYZI>::Ptr transcloud(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f keypose_matrix;

    RPYpose rpy_pose;
    rpy_pose.x  = trans_pose.x;
    rpy_pose.y  = trans_pose.y;
    rpy_pose.z  = trans_pose.z;
    rpy_pose.roll  = trans_pose.roll;
    rpy_pose.pitch  = trans_pose.pitch;
    rpy_pose.yaw  = trans_pose.yaw;

    RPYposeToMatrix(rpy_pose,keypose_matrix);

    Eigen::Matrix4f final_matrix =  align_matrix * keypose_matrix;
    pcl::transformPointCloud(*mMapDataFrame->start_keyframe_ptr,*transcloud,align_matrix);
//    pcl::io::savePCDFileBinaryCompressed("start2.pcd",*transcloud);
    transcloud->clear();
    pcl::transformPointCloud(mMapDataFrame->cloud_keyframes_v[start_id],*transcloud,final_matrix);

//    pcl::io::savePCDFileBinaryCompressed("start1.pcd",*transcloud);
//    pcl::io::savePCDFileBinaryCompressed("end.pcd",*mMapDataFrame->end_keyframe_ptr);

    UpdateFitnessScore(align_matrix);
    return final_matrix;
}

void AUTOMATCHING::UpdateFitnessScore(Eigen::Matrix4f replace)
{
    fitness_score_ = CalcFitnessScore(mMapDataFrame->end_keyframe_ptr, mMapDataFrame->start_keyframe_ptr, replace, 1.0);
    fitness_score_ = std::min(1000000.0, fitness_score_);
}
