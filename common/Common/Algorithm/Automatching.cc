/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     Automatching.cc
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:05:49
*/


/**************************************************************************************************/
#include "Common/Algorithm/Automatching.hh"
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d_omp.h>
#include "Common/Algorithm/match_core/ndt/ndt_omp.h"
#include "Common/Algorithm/match_core/ndt/gicp_omp.h"
#include <pcl/registration/gicp.h>
#include "Common/Algorithm/match_core/gpicp/gpicp_omp.h"
#include "Common/Algorithm/match_core/gpicp/gpicp.h"
#include "Common/Algorithm/match_core/include/fast_gicp/gicp/fast_gicp.hpp"
#include "Common/Algorithm/match_core/include/fast_gicp/gicp/fast_vgicp.hpp"
//#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#include "Common/Algorithm/m2dp.h"
#include "pcl/registration/icp.h"

//debug <<--
#include <QString>
//-->>

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
    matchingscore = new MatchingScore();
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

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_start_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setLeafSize(0.4,0.4,0.4);
    filter.setInputCloud(mMapDataFrame->start_keyframe_ptr);
    filter.filter(*filter_start_point);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_end_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> filter2;
    filter2.setLeafSize(0.4,0.4,0.4);
    filter2.setInputCloud(mMapDataFrame->end_keyframe_ptr);
    filter2.filter(*filter_end_point);

    pcl::copyPointCloud(*filter_start_point, *start_keyframe_cloud);
    pcl::copyPointCloud(*filter_end_point, *end_keyframe_cloud);

    std::cout << "start_keyframe_ptr size  : " << mMapDataFrame->start_keyframe_ptr->size() << std::endl;
    std::cout << "end_keyframe_ptr size  : " << mMapDataFrame->end_keyframe_ptr->size() << std::endl;
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
    ReadMapEditConfig();
    mMapDataFrame->start_keyframe_ptr->clear();
    mMapDataFrame->end_keyframe_ptr->clear();
    mMapDataFrame->start_keyframe_ptr = GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[start_id].makeShared(),trans_pose);

    if(!bsubmap)
        mMapDataFrame->end_keyframe_ptr = GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[end_id].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[end_id]);
    else
        *mMapDataFrame->end_keyframe_ptr += *mMapDataFrame->near_history_keyframes_ptr;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_start_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setLeafSize(0.4,0.4,0.4);
    filter.setInputCloud(mMapDataFrame->start_keyframe_ptr);
    filter.filter(*filter_start_point);

    //debug <<--
//    std::cout << "===========start keyframe:" << std::endl;
//    for(int i = 0; i < filter_start_point->size(); ++i)
//    {
//        std::cout << filter_start_point->at(i).x << ", " << filter_start_point->at(i).y << ", " << filter_start_point->at(i).z << std::endl;
//    }

//    std::cout << std::endl << std::endl;
    //-->>

    pcl::PointCloud<pcl::PointXYZI>::Ptr filter_end_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> filter2;
    filter2.setLeafSize(0.4,0.4,0.4);
    filter2.setInputCloud(mMapDataFrame->end_keyframe_ptr);
    filter2.filter(*filter_end_point);

    Eigen::Matrix4f align_matrix;
    auto t1 = std::chrono::high_resolution_clock::now();
    if(match_type=="ndt")
    {
        std::cout << "ndt matching" << std::endl;
        pclomp::NormalDistributionsTransform<PointType3D, PointType3D>::Ptr NdtAligner;
        NdtAligner.reset(new pclomp::NormalDistributionsTransform<PointType3D, PointType3D>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        NdtAligner->setMaximumIterations(64);
        NdtAligner->setNumThreads(7);
        NdtAligner->setNeighborhoodSearchMethod(pclomp::DIRECT1);
        NdtAligner->setInputTarget(filter_end_point);
        NdtAligner->setTransformationEpsilon(0.01);
        NdtAligner->setStepSize(0.1);
        NdtAligner->setResolution(1.0);
        NdtAligner->setInputSource(filter_start_point);
        NdtAligner->align(*aligned);
        align_matrix = NdtAligner->getFinalTransformation();
    }

    if(match_type=="gicp")
    {
        std::cout << "gicp matching" << std::endl;

        pclomp::GeneralizedIterativeClosestPoint<PointType3D, PointType3D>::Ptr gicp;
        gicp.reset(new pclomp::GeneralizedIterativeClosestPoint<PointType3D, PointType3D>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        gicp->setMaximumIterations(100);
        gicp->setInputTarget(filter_end_point);
        gicp->setInputSource(filter_start_point);
        gicp->setTransformationEpsilon(1e-6);
        gicp->setEuclideanFitnessEpsilon(1e-6);
        gicp->align(*aligned);
        align_matrix = gicp->getFinalTransformation();
    }

    if(match_type=="fgicp")
    {
        std::cout << "fgicp matching" << std::endl;
        boost::shared_ptr<fast_gicp::FastGICP<PointType3D, PointType3D>> fgicp(new fast_gicp::FastGICP<PointType3D, PointType3D>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        fgicp->setNumThreads(0);
        fgicp->setTransformationEpsilon(0.01);
        fgicp->setMaximumIterations(64);
        fgicp->setMaxCorrespondenceDistance(2.5);
        fgicp->setCorrespondenceRandomness(20);
        fgicp->setInputTarget(filter_end_point);
        fgicp->setInputSource(filter_start_point);
        fgicp->align(*aligned);
        align_matrix = fgicp->getFinalTransformation();
    }

    if(match_type=="vgicp")
    {
        std::cout << "vgicp matching" << std::endl;
        boost::shared_ptr<fast_gicp::FastVGICP<PointType3D, PointType3D>> vgicp(new fast_gicp::FastVGICP<PointType3D, PointType3D>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        vgicp->setNumThreads(0);
        vgicp->setResolution(1.0);
        vgicp->setTransformationEpsilon(0.01);
        vgicp->setMaximumIterations(64);
        vgicp->setCorrespondenceRandomness(20);
        vgicp->setInputTarget(filter_end_point);
        vgicp->setInputSource(filter_start_point);
//        vgicp->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
        vgicp->align(*aligned);
        align_matrix = vgicp->getFinalTransformation();
    }

//    if(match_type=="fgicpcuda")
//    {
//        std::cout << "vgicp matching" << std::endl;
//        boost::shared_ptr<fast_gicp::FastVGICPCuda<PointType3D, PointType3D>> vgicp(new fast_gicp::FastVGICPCuda<PointType3D, PointType3D>());
//        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
//        vgicp->setResolution(1.0);
//        vgicp->setTransformationEpsilon(0.01);
//        vgicp->setMaximumIterations(64);
//        vgicp->setCorrespondenceRandomness(20);
//        vgicp->setInputTarget(filter_end_point);
//        vgicp->setInputSource(filter_start_point);
//        vgicp->align(*aligned);
//        align_matrix = vgicp->getFinalTransformation();
//    }

    if(match_type=="gpicp")
    {
        std::cout << "gpicp matching" << std::endl;
        pclomp::GeneralizedIterativeClosestPoint_GP<PointType3D, PointType3D>::Ptr gpicp;
        gpicp.reset(new pclomp::GeneralizedIterativeClosestPoint_GP<PointType3D, PointType3D>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        gpicp->setMaximumIterations(100);
        gpicp->setInputTarget(filter_end_point);
        gpicp->setInputSource(filter_start_point);
        gpicp->setTransformationEpsilon(1e-6);
        gpicp->setEuclideanFitnessEpsilon(1e-6);
        gpicp->align(*aligned);
        align_matrix = gpicp->getFinalTransformation();
    }

//    //debug <<--
//    std::cout << "=============cc align_matrix:" << std::endl;
//    std::cout << QString::number(align_matrix(0, 0), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(0, 1), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(0, 2), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(0, 3), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(1, 0), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(1, 1), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(1, 2), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(1, 3), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(2, 0), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(2, 1), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(2, 2), 'f', 6).toStdString() << " "
//              << QString::number(align_matrix(2, 3), 'f', 6).toStdString() << std::endl;
//    //-->>

    auto t2 = std::chrono::high_resolution_clock::now();

    double single = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
    std::cout << "match time :" << single << " ms " << std::flush;

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
//    //debug <<--
//    std::cout << "=============cc keypose_matrix:" << std::endl;
//    std::cout << QString::number(keypose_matrix(0, 0), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(0, 1), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(0, 2), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(0, 3), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(1, 0), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(1, 1), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(1, 2), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(1, 3), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(2, 0), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(2, 1), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(2, 2), 'f', 6).toStdString() << " "
//              << QString::number(keypose_matrix(2, 3), 'f', 6).toStdString() << std::endl;
//    //-->>

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


void AUTOMATCHING::FitnessScore(Eigen::Matrix4f replace)
{
    fitness_score_ = CalcFitnessScore(mMapDataFrame->near_history_keyframes_ptr, latest_keyframe_.makeShared(),replace, 1.0);
    fitness_score_ = std::min(1000000.0, fitness_score_);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr AUTOMATCHING::transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in, const PointXYZIRPYT &trans)
{
    Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
    this_transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ()) *
                                             Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
                                             Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
    this_transformation(0, 3) = trans.x;
    this_transformation(1, 3) = trans.y;
    this_transformation(2, 3) = trans.z;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud_in, *tf_cloud, this_transformation);
    return tf_cloud;
}

bool AUTOMATCHING::AutoLoopclousing(float search_radius,int index_detect, float fitness_score_threshold, float match_score_threshold, int latest_history_frame_id,int *closest_id_,RPYpose *pose, double &fitness_score, double &match_score)
{
    //    for(int i=first_id;i<mMapDataFrame->cloud_keyposes_6d_ptr->size();i++)
    std::vector<int> search_idx_;
    std::vector<float> search_dist_;

    latest_keyframe_.clear();
    mMapDataFrame->near_history_keyframes_ptr->clear();

    double x_last = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].x ;
    double y_last = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].y ;
    //std::cout << "latest_history_frame_id :  " << latest_history_frame_id  << " x_last : " << x_last <<   "y_last : " << y_last << std::endl;

    pcl::PointXY cur_pose;
    cur_pose.x = x_last;
    cur_pose.y = y_last;

    mMapDataFrame->kdtree_pose_2d->setInputCloud(mMapDataFrame->cloud_keyposes_2d_ptr);
    //std::cout << "mMapDataFrame->cloud_keyposes_2d_ptr size  : " << mMapDataFrame->cloud_keyposes_2d_ptr->size() << std::endl;
    mMapDataFrame->kdtree_pose_2d->radiusSearch(cur_pose, search_radius, search_idx_, search_dist_);

    int current_id = latest_history_frame_id;
    static int previous_id = current_id;
    double diff_id = current_id - previous_id ;
    int closest_history_frame_id= -1;
    double min_dist=100000000.0;
    int history_search_num_ =10;


    for (int j = 0; j < search_idx_.size(); ++j)
    {
        //        std::cout << "latest_history_frame_id_ - search_idx_[i] :  " << latest_history_frame_id_ - search_idx_[i] << std::endl;
        if (fabs(latest_history_frame_id - search_idx_[j]) > index_detect)
        {
            double x_closed = mMapDataFrame->cloud_keyposes_6d_ptr->points[search_idx_[j]].x;
            double y_closed = mMapDataFrame->cloud_keyposes_6d_ptr->points[search_idx_[j]].y;
            double dist=sqrt((x_last-x_closed)*(x_last-x_closed)+(y_last-y_closed)*(y_last-y_closed));
            if(dist<min_dist)
            {
                closest_history_frame_id = search_idx_[j];
                min_dist=dist;
            }
        }
    }

    //std::cout << "closest_history_frame_id : " << closest_history_frame_id << std::endl;

    if(closest_history_frame_id==-1)
        return false;

    previous_id = current_id;

    double height_last = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].z;
    double height_history = mMapDataFrame->cloud_keyposes_6d_ptr->points[closest_history_frame_id].z;
    double diff_height = height_last - height_history;

    RPYpose pose_trans;
    pose_trans.roll = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].roll;
    pose_trans.pitch = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].pitch;
    pose_trans.yaw = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].yaw;
    pose_trans.x = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].x ;
    pose_trans.y =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].y ;
    pose_trans.z =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].z - diff_height;

    Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
    RPYposeToMatrix(pose_trans,this_transformation);

    pcl::transformPointCloud(mMapDataFrame->cloud_keyframes_v[latest_history_frame_id], latest_keyframe_, this_transformation);
    //        pcl::PointCloud<>
    mMapDataFrame->ds_source_.setInputCloud(latest_keyframe_.makeShared());
    mMapDataFrame->ds_source_.filter(latest_keyframe_);

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = -history_search_num_, j; i <= history_search_num_; ++i)
    {
        j = closest_history_frame_id + i;
        if (j < 0 /*|| j >= latest_history_frame_id_*/)
        {
            return false;
        }

        if(j>mMapDataFrame->cloud_keyframes_v.size()-1)
        {
            return false;
        }

        //从硬盘加载对应的keyframe
        std::string cloud_path = kf_path + std::to_string(j) + ".pcd";
        //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::loadPCDFile(cloud_path.c_str(), mMapDataFrame->cloud_keyframes_v[j]);
        *tmp_cloud += *transformPointCloud(mMapDataFrame->cloud_keyframes_v[j].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[j]);
    }

    pcl::io::savePCDFileBinaryCompressed("tmp.pcd",*tmp_cloud);

    mMapDataFrame->ds_history_keyframes_.setInputCloud(tmp_cloud);
    mMapDataFrame->ds_history_keyframes_.filter(*mMapDataFrame->near_history_keyframes_ptr);

    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> registration_loop;

    Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f correction_frame;
    pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());

    bool use_ndt_loop_match=false;
    bool has_converged=false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_keyframe_trans(new pcl::PointCloud<pcl::PointXYZI>);

//    double fitness_score=-1;

    //std::cout << "进入 ndt loop matching----------------------\n";
    use_ndt_loop_match =true;
    registration_loop.setTransformationEpsilon(0.01);
    registration_loop.setResolution(2.0);
    registration_loop.setStepSize(0.5);

    //#if COMPUTE_MATCHING ==1
    registration_loop.setNeighborhoodSearchMethod(pclomp::DIRECT1);
    registration_loop.setNumThreads(1);
    registration_loop.setMaximumIterations(64);
    registration_loop.setInputTarget(mMapDataFrame->near_history_keyframes_ptr);
    registration_loop.setInputSource(latest_keyframe_.makeShared());
    pcl::io::savePCDFileBinaryCompressed("near_history_keyframes_ptr.pcd",*mMapDataFrame->near_history_keyframes_ptr);

    RPYpose rpy_pose;
    rpy_pose.x  = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].x;
    rpy_pose.y  = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].y;
    rpy_pose.z  = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].z-diff_height;
    rpy_pose.roll  = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].roll;
    rpy_pose.pitch  = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].pitch;
    rpy_pose.yaw  = mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id].yaw;

    RPYposeToMatrix(rpy_pose,initial_guess);
    //#if COMPUTE_MATCHING ==1
    registration_loop.align(*unused_result);
    has_converged = registration_loop.hasConverged();

    correction_frame = registration_loop.getFinalTransformation();

    latest_keyframe_trans->clear();

    pcl::transformPointCloud(latest_keyframe_,*latest_keyframe_trans,correction_frame);
    FitnessScore(correction_frame/*.inverse()*/);



    Eigen::Matrix4f t_correct = correction_frame * initial_guess;

//    shift_loop = std::sqrt(std::pow(t_correct(0, 3) -mMapDataFrame->cloud_keyposes_6d_ptr->points[closest_history_frame_id_].x , 2)
//                           + std::pow(t_correct(1, 3) -mMapDataFrame->cloud_keyposes_6d_ptr->points[closest_history_frame_id_].y, 2));
    //std::cout << "fitness_score : " << fitness_score_ << std::endl;
    //std::cout << "has_converged : " << has_converged << std::endl;

    stringstream last_id;
    stringstream closet_id;



    //        processMatchingScore(latest_keyframe_trans,near_history_keyframes_);
    fitness_score = fitness_score_;
    match_score = matchingscore->ProcessMatchScore(latest_keyframe_trans,mMapDataFrame->near_history_keyframes_ptr);

    //std::cout << "match socre : " << match_score << std::endl;
    if((has_converged == false ) || (fitness_score_>fitness_score_threshold) || (match_score<match_score_threshold) )
    {
        printf("检测到闭环点，但是没有满足约束条件:\n,"
               "has_converged=%d, fitness_score_loop=%f,latest keyframe size=%d,闭环点id=%d， 当前点id=%d \n",has_converged,fitness_score_,latest_keyframe_.size(),closest_history_frame_id,latest_history_frame_id);
        return false;
    }

    RPYpose out_pose;

    MatrixToRPYpose(t_correct,out_pose);
    *pose = out_pose;
    *closest_id_ = closest_history_frame_id;
    return true;
}
