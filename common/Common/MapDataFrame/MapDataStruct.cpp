/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     MapDataStruct.cpp
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:10:15
*/
#include <Common/MapDataFrame/MapDataStruct.hh>
MAPDATAFRAME::MAPDATAFRAME()
{
//    loop_correct_closed = false;
    AllMapDataFrameMemory();
    LoopClosingMapFrameMemory();
    is_increment_request = false;
    human_correct_pose = false; // posemapping
    interactive_mode  = 0;
    float ds_history_size_ =0.4;
    ds_source_.setLeafSize(ds_history_size_, ds_history_size_, ds_history_size_);
    ds_history_keyframes_.setLeafSize(ds_history_size_, ds_history_size_, ds_history_size_);
    std::cout << "AllMapDataFrameMemory " << std::endl;
}

MAPDATAFRAME::~MAPDATAFRAME()
{

}

void MAPDATAFRAME::AllMapDataFrameMemory()
{
//    kdtree_global_map_ptr.reset(new pcl::KdTreeFLANN<PointType3D>);
    final_3D_map_cloud_ptr.reset(new pcl::PointCloud<PointType3D>);
    final_2D_map_cloud_ptr.reset(new pcl::PointCloud<PointType3D>);
    cloud_keyposes_3d_ptr.reset(new pcl::PointCloud<PointType3D>);
    cloud_keyposes_2d_ptr.reset(new pcl::PointCloud<PointType2D>);
    cloud_keyposes_6d_ptr.reset(new pcl::PointCloud<PointType6D>);
    lidar_scan_ptr.reset(new pcl::PointCloud<PointType3D>);
    target_subMap_ptr.reset(new pcl::PointCloud<PointType3D>);

    kdtree_pose_2d.reset(new pcl::KdTreeFLANN<pcl::PointXY>);
    global_2dMapKeyframes_ptr.reset(new pcl::PointCloud<PointType3D>);
    global_2dMapKeyframes_ds_ptr.reset(new pcl::PointCloud<PointType3D>);

    global_3dMapKeyframes_ptr.reset(new pcl::PointCloud<PointType3D>);
    global_3dMapKeyframes_ds_ptr.reset(new pcl::PointCloud<PointType3D>);
    start_keyframe_ptr.reset(new pcl::PointCloud<PointType3D>);
    end_keyframe_ptr.reset(new pcl::PointCloud<PointType3D>);
    std::vector<pcl::PointCloud<PointType3D>>().swap(cloud_keyframes_v);

}

void MAPDATAFRAME::LoopClosingMapFrameMemory()
{
    cloud_loop_index.reset(new pcl::PointCloud<Pointlooppose>);
    latest_keyframe_ptr.reset(new pcl::PointCloud<PointType3D>);
    near_history_keyframes_ptr.reset(new pcl::PointCloud<PointType3D>);
    cloud_pose_graph_ptr.reset(new pcl::PointCloud<Pointlooppose>);
//    kdtree_pose_2d_ptr.reset(new pcl::KdTreeFLANN<PointType2D>);
}

