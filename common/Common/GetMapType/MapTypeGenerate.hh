/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     MapTypeGenerate.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:07:14
*/
#ifndef MAPTYPEGENERATE_HH
#define MAPTYPEGENERATE_HH
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <Common/MapDataFrame/MapDataStruct.hh>
#include <Common/Commonfig.hh>
#include <Common/ParamServer.hh>
#include <Common/Algorithm/ground_filter/ground_filter.hh>
#include <Common/Algorithm/probabilitymap/probability_map.h>
#include <boost/serialization/singleton.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <tuple>

#include "obstaclemap.h"

struct  SaveConfig
{
    int save_single_size_;
    double tile_size_;
    int grid_size_;
    double map_voxel_size_;
    bool use_map_filter_;
    bool use_ground_filter_;
    bool save_debug_data_;
    bool user_cut_save_type_;
    std::vector<std::tuple<int, int>> inner_;
    std::vector<std::tuple<int, int>> outer_;
    std::string save_path_;
    std::string site_name_;
    bool save_2d_;
    bool save_map_tile_;
    bool save_prob_grid_map_;
    bool save_segmap_;
    float car_height_;
    int skip_interval_;
    bool save_submap_;
    float save_submap_filter_;
    std::vector<std::tuple<int, int>> submap_inner_;
    std::vector<std::tuple<int, int>> submap_outer_;
    bool save_singlescan_map_; //true为单线保存，false为多线保存


    SaveConfig() : save_single_size_(1000),
        tile_size_(35.5),
        grid_size_(10),
        map_voxel_size_(0.2),
        use_map_filter_(true),
        use_ground_filter_(true),
        save_debug_data_(true),
        save_2d_(true),
        save_map_tile_(true),
        save_prob_grid_map_(true),
        car_height_(0.0)
    {

    }

};

class MAPDATAFRAME;
class MAPTYPEGENERATE
{
public:
    MAPTYPEGENERATE(MAPDATAFRAME* MapDataFrame, SaveConfig *save_config);
    ~MAPTYPEGENERATE();
    MAPDATAFRAME *MapFrame;
    void RebuildCloudMap(pcl::PointCloud<pcl::PointXYZI>& cloud_map);

    bool GetFolder();
    void Get2D3DGlobalCloud();
    void SaveTileMap();
    void Save2DMap(int &progress, std::vector<std::tuple<int, int>> inner, std::vector<std::tuple<int, int>> outer);
    void SaveGridMap();
    void saveProbGridMap(int &progress);
    void Update2DMap(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn);
    void Update2DMap(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground);
    void WriteConfig();
    void Load3dPath();
    void CSF_groundFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input);
//    void ReadConfig();

    cv::Mat ConvertToRainbow(const cv::Mat& scaledGray);
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_map_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_2dmap_ptr;

    MapProp MapRange;
    cv::Mat TotalMap, TotalIntensityMap, TotalMap_tmp,CountToalMap,TotalMixMap;
//    float GridSize ;

    string savePath;
    string sub_save_2DPath;
    string sub_save_PosePath;
    string sub_save_MaptilePath;
    string sub_save_keyframePath;

    std::vector<std::string> vCloudfilename2;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeAllMap;

    GROUNDFILTER *Ground_filter;

    std::vector<mapIndex> grids;

    void TileMapInit();
    void TileMapInit(pcl::PointCloud<PointType3D>::Ptr cloud_keyposes_3d_ptr);
    void SaveTileMapCache();

    int delta_allInd_size_x;
    int delta_allInd_size_y;
    int ind_x_min;
    int ind_y_min;
    int ind_x_max;
    int ind_y_max;

    int grid_num;

    //lyx add 2023-2-9 <<--
    void save_submap();
    //-->>

//    pcl::VoxelGrid<pcl::PointXYZI> downSizeAllMap;


//private:
//    pcl::PointCloud<PointType3D>::Ptr GetTransformPoint2DCloud(pcl::PointCloud<PointType3D> cloudIn, Eigen::Matrix4f Trans, float car_height);


protected:
    MAPDATAFRAME* mMapDataFrame;
    SaveConfig *save_config_;

    //seg
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_seeds_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_ground_pc,seg_ground_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_not_ground_pc,seg_not_ground_pc;
    pcl::PointCloud<PointXYZIL>::Ptr g_all_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw;
    pcl::PointCloud<PointXYZRGBIL>::Ptr cloud_segmap;
    pcl::PointCloud<Pointlooppose>::Ptr cloud_pose_graph;
    ObstacleMap *myobstaclemap;

    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_;
    int min_x_;
    int min_y_;
    int max_x_;
    int max_y_;
};
#endif
