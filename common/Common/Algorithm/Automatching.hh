/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     Automatching.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:08:04
*/
#ifndef AUTOMATCHING_HH
#define AUTOMATCHING_HH
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <Common/MapDataFrame/MapDataStruct.hh>
#include <Common/Commonfig.hh>
#include <Common/ParamServer.hh>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/kdtree.hpp>
#include "match_core/ndt/match_score.hh"
using FeatureT = pcl::FPFHSignature33;

class AUTOMATCHING
{
public:
    AUTOMATCHING(MAPDATAFRAME *MapDataFrame);
    ~AUTOMATCHING();
    Eigen::Matrix4f AutoMatching(int start_id,int end_id);
    Eigen::Matrix4f ScanMatching(int start_id,int end_id,PointType6D trans_pose);
    void UpdateFitnessScore(Eigen::Matrix4f replace);
    void FitnessScore(Eigen::Matrix4f replace);
    bool AutoLoopclousing(float search_radius,int index_detect, float fitness_score_threshold, float match_score_threshold, int latest_history_frame_id,int *closest_id_,RPYpose *pose, double &fitness_score, double &match_score);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in, const PointXYZIRPYT &trans);
    double fitness_score_;
    MatchingScore* matchingscore;
    pcl::PointCloud<pcl::PointXYZI> latest_keyframe_;

private:
    MAPDATAFRAME* mMapDataFrame;
    float fpfh_normal_estimation_radius;
    float fpfh_search_radius;
    int fpfh_max_iterations;
    int fpfh_num_samples;
    int fpfh_correspondence_randomness;
    float fpfh_similarity_threshold;
    float fpfh_max_correspondence_distance;
    float fpfh_inlier_fraction;

};
#endif
