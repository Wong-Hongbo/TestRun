/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     MapDataStruct.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:06:37
*/
#ifndef MAP_DATA_STRUCT_HH
#define MAP_DATA_STRUCT_HH
#include "pcl/common/common.h"
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <Common/Commonfig.hh>
#include <pcl/kdtree/kdtree_flann.h>
#include <mutex>
#include <pcl/filters/voxel_grid.h>
//#include <ccPointCloud.h>

class MAPDATAFRAME
{
public:
    MAPDATAFRAME();
    MAPDATAFRAME(const MAPDATAFRAME &other)
    {}
    ~MAPDATAFRAME();

    //loop detect
    pcl::PointCloud<PointType3D>::Ptr near_history_keyframes_ptr;
    std::deque<pcl::PointCloud<PointType3D>::Ptr> recent_keyframes_ptr;
//    pcl::KdTreeFLANN<PointType3D>::Ptr kdtree_global_map_ptr;
    pcl::PointCloud<PointType3D>::Ptr latest_keyframe_ptr;
//    pcl::KdTreeFLANN<PointType2D>::Ptr kdtree_pose_2d_ptr;

    //keyframe data
    pcl::PointCloud<PointType3D>::Ptr cloud_keyposes_3d_ptr;
    pcl::PointCloud<PointType2D>::Ptr cloud_keyposes_2d_ptr;
    pcl::PointCloud<PointType6D>::Ptr cloud_keyposes_6d_ptr;
    pcl::PointCloud<PointType3D>::Ptr cloud_keypose_3d_rtk_ptr;
    pcl::PointCloud<PointTGPS>::Ptr cloud_keypose_6d_rtk_ptr;
//    ccPointCloud *cc_point_cloud_looppose;
    std::vector<pcl::PointCloud<PointType3D>> cloud_keyframes_v;
    //std::vector<ccPointCloud*> cc_point_cloud_keyframes_v;
//    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> recent_keyframes_ptr;

    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_pose_2d;
    //global Mapdata
    pcl::PointCloud<PointType3D>::Ptr global_2dMapKeyframes_ptr;
    pcl::PointCloud<PointType3D>::Ptr global_2dMapKeyframes_ds_ptr;

    pcl::PointCloud<PointType3D>::Ptr global_3dMapKeyframes_ptr;
    pcl::PointCloud<PointType3D>::Ptr global_3dMapKeyframes_ds_ptr;

    //MapType
    pcl::PointCloud<PointType3D>::Ptr final_3D_map_cloud_ptr;
    pcl::PointCloud<PointType3D>::Ptr final_2D_map_cloud_ptr;

    //frame
    pcl::PointCloud<PointType3D>::Ptr lidar_scan_ptr;
//    pcl::PointCloud<PointType3D>::Ptr filter_lidar_scan_ptr;

    //submap
    pcl::PointCloud<PointType3D>::Ptr target_subMap_ptr;

    pcl::PointCloud<PointType3D>::Ptr start_keyframe_ptr;
    pcl::PointCloud<PointType3D>::Ptr end_keyframe_ptr;

    pcl::PointCloud<Pointlooppose>::Ptr cloud_loop_index;  // 建图数据使用的looppose.pcd，需要继续保留使用，其他的功能不适用
    pcl::PointCloud<Pointlooppose>::Ptr cloud_pose_graph_ptr; // 关键帧数据

    RPYpose current_rebot_pose;

    int latest_history_frame_id;   //start point
    int closest_history_frame_id;  //end  point

    void AllMapDataFrameMemory();
    void LoopClosingMapFrameMemory();
    std::mutex mtx_;
    std::mutex show_lck;
    bool loop_correct_closed;
    bool is_increment_request;
    int first_load_odom_size;
    int first_load_loop_size;
    bool human_correct_pose; //
    Eigen::Matrix4f add_init_matrix;
    std::string keyframe_path;
    int interactive_mode;  //1:pose map  0: normal map
    pcl::VoxelGrid<PointT> ds_source_;
    pcl::VoxelGrid<PointT> ds_history_keyframes_;


};


#endif
