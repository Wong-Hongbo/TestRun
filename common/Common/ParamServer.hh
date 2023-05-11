/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     ParamServer.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:06:50
*/
#ifndef PARAM_SERVER_HH_
#define PARAM_SERVER_HH_
#include <Common/XSTF/LinearMath/Transform.h>
#include <Common/XSTF/LinearMath/Quaternion.h>
#include <Common/Commonfig.hh>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <HDLadarDatan.hh>
#include <boost/serialization/singleton.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
inline const int rtk_dev = 150;
inline const int poseCovThreshold = 25;
inline const float history_search_radius = 12;
inline const std::vector<int> search_idx_;
inline const std::vector<float> search_dist_;

// 地图偏移信息
inline float pose_x; //cm
inline float pose_y; //cm

inline std::string kf_path;
inline std::string seg_kf_path;
inline int ugv_zone;
inline float ugv_name;
inline float score;

inline double station_lon;
inline double station_lat;
inline double station_height;
inline int map_type;

inline bool b_show2d;
inline bool b_show3d;
inline float ugv_height;
inline bool use_backpack;
inline float lidar_max_range;
inline bool use_dynatic_filter;
inline float scan_voxel_size;
inline float voxel_size_step;
inline float min_voxel_size;
inline float max_voxel_size;
inline float min_pointsize;
inline float max_pointsize;
inline int save_single_size;
inline float tile_size;
inline float grid_size;
inline float map_voxel_size;
inline bool use_map_filter;
inline bool use_ground_filter;
inline bool save_debug_data;
inline bool use_cut_save_map;
inline int  save_sub_start_id;
inline int  save_sub_end_id;
inline bool use_gps;
inline bool use_loop_detect;
inline bool use_manual_lc;
inline int  odom_type;
inline int search_map_num;
inline float kf_dis;
inline int loop_match_tpye;
inline float search_loop_radius;
inline float lc_score;
inline bool bsubmap  = true;
inline string match_type = "gpicp";

inline std::deque<HDLADARDATA_MSG*> messageQueue;
//inline RPYpose pre_keypose;
inline void MatrixToRPYpose(const Eigen::Matrix4f matrix, RPYpose& pose)
{
    XSTF::Matrix3x3 mat33;

    mat33.setValue(static_cast<double>(matrix(0, 0)), static_cast<double>(matrix(0, 1)),
                   static_cast<double>(matrix(0, 2)), static_cast<double>(matrix(1, 0)),
                   static_cast<double>(matrix(1, 1)), static_cast<double>(matrix(1, 2)),
                   static_cast<double>(matrix(2, 0)), static_cast<double>(matrix(2, 1)),
                   static_cast<double>(matrix(2, 2)));

    pose.x = matrix(0, 3);
    pose.y = matrix(1, 3);
    pose.z = matrix(2, 3);

    mat33.getRPY(pose.roll, pose.pitch, pose.yaw, 1);
    pose.yaw = pose.yaw ;
}

inline void RPYposeToMatrix(const RPYpose pose, Eigen::Matrix4f &matrix)
{
    Eigen::AngleAxisf rotation_pitch(pose.roll, Eigen::Vector3f::UnitX()); // roll
    Eigen::AngleAxisf rotation_roll(pose.pitch, Eigen::Vector3f::UnitY()); // pitch
    Eigen::AngleAxisf rotation_yaw(pose.yaw, Eigen::Vector3f::UnitZ());    // yaw
    Eigen::Translation3f translation(pose.x, pose.y, pose.z);
    matrix = (translation * rotation_yaw * rotation_roll * rotation_pitch).matrix();
}


inline pcl::PointCloud<PointType3D>::Ptr GetTransformPoint3DCloud(const pcl::PointCloud<PointType3D>::ConstPtr cloud_in, const PointType6D &trans)
{
    Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
    this_transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ()) *
                                             Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
                                             Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
    this_transformation(0, 3) = trans.x;
    this_transformation(1, 3) = trans.y;
    this_transformation(2, 3) = trans.z;
    pcl::PointCloud<PointType3D>::Ptr tf_cloud(new pcl::PointCloud<PointType3D>());
    pcl::transformPointCloud(*cloud_in, *tf_cloud, this_transformation);
    return tf_cloud;
}

inline pcl::PointCloud<PointType3D>::Ptr GetTransformPoint2DCloud(pcl::PointCloud<PointType3D> cloudIn, Eigen::Matrix4f Trans)
{
    PointType3D p;
    int cloudSize = cloudIn.points.size();
    pcl::PointCloud<PointType3D>::Ptr cloudOut(new pcl::PointCloud<PointType3D>);
    double x,y,z;
    double height_ref;
    for (int i = 0; i < cloudSize; ++i)
    {
        x = cloudIn.points[i].x;
        y = cloudIn.points[i].y;
        z = cloudIn.points[i].z + ugv_height;
        int intensity = int(cloudIn.points[i].intensity);

        if(sqrt(x*x+y*y)>30)
            continue;
        if(x!=0 || y!=0 )
        {
            if(x>-2.0 && x<2.0 && y<=3.0 && y>=-3)
                continue;
            if(z<-1.0 || z>2.2)    //if(z<-1 || z>15)
                continue;
            if(y<-15 || y>15)   continue;
            p.x = Trans(0,0)*x +Trans(0,1)*y +Trans(0,2)*z +Trans(0,3);
            p.y = Trans(1,0)*x +Trans(1,1)*y +Trans(1,2)*z +Trans(1,3);
            height_ref = Trans(2,0)*x +Trans(2,1)*y +Trans(2,2)*z +Trans(2,3);
            p.x = p.x*100.0f ;  //cm
            p.y = p.y*100.0f ;
            p.z = z;
            //            p.z = height_ref  - Trans(2,3);
            p.intensity = intensity;
            cloudOut->push_back(p);
        }
    }
    return cloudOut;
}


inline double CalcFitnessScore(const pcl::PointCloud<PointType3D>::Ptr& cloud1, const pcl::PointCloud<PointType3D>::Ptr& cloud2, const Eigen::Matrix4f& relpose, double max_range)
{
    pcl::search::KdTree<PointType3D>::Ptr tree_(new pcl::search::KdTree<PointType3D>());
    tree_->setInputCloud(cloud1);

    double fitness_score = 0.0;

    // Transform the input dataset using the final transformation
    pcl::PointCloud<PointType3D> input_transformed;
    pcl::transformPointCloud (*cloud2, input_transformed, relpose);

    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    // For each point in the source dataset
    int nr = 0;
    for (size_t i = 0; i < input_transformed.points.size (); ++i)
    {
        // Find its nearest neighbor in the target
        tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= max_range)
        {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (nr > 0)
        return (fitness_score / nr);
    else
        return (std::numeric_limits<double>::max ());
}

inline RPYpose PclToRPYpose(PointType6D pose)
{
    RPYpose pose_rpy;
    pose_rpy.x =  pose.x;
    pose_rpy.y =  pose.y;
    pose_rpy.z =  pose.z;
    pose_rpy.roll =  pose.roll;
    pose_rpy.pitch =  pose.pitch;
    pose_rpy.yaw =  pose.yaw;
    return pose_rpy;
}

inline PointType3D RPYposePcl(RPYpose pose)
{
    PointType3D pose_pcl;
    pose_pcl.x =  pose.x;
    pose_pcl.y =  pose.y;
    pose_pcl.z =  pose.z;
    return pose_pcl;
}

inline void ReadMapEditConfig()
{
    string home_path = getenv("HOME");
    std::string ugv_ini = home_path + "/.MappingEdit/MapEditConfig.ini";
    boost::property_tree::ptree tag_setting;
    boost::property_tree::ini_parser::read_ini(ugv_ini, tag_setting);

    std::cout << "#[SHOW] config start #" << std::endl;
    b_show2d = tag_setting.get<bool>("SHOW.SHOW2DFLAG");
    std::cout << "show 2d : " << b_show2d << std::endl;
    b_show3d = tag_setting.get<bool>("SHOW.SHOW3DFLAG");
    std::cout << "show 3d : " << b_show3d << std::endl;
    std::cout << "#[SHOW] config end #" << std::endl;
    std::cout << " " << std::endl;

    std::cout << "#[DATATYPE] config start #" << std::endl;
    ugv_height = tag_setting.get<float>("DATATYPE.UGVHEIGHT");
    std::cout << "ugv_height : " << ugv_height << std::endl;
    use_backpack = tag_setting.get<bool>("DATATYPE.USEBACKPACK");
    std::cout << "use_backpack : " << use_backpack << std::endl;
    lidar_max_range = tag_setting.get<float>("DATATYPE.LIDARMAXRANGE");
    std::cout << "lidar_max_range : " << lidar_max_range << std::endl;
    use_dynatic_filter = tag_setting.get<bool>("DATATYPE.USEDYNATICSCAN");
    std::cout << "use_dynatic_filter : " << use_dynatic_filter << std::endl;
    scan_voxel_size = tag_setting.get<float>("DATATYPE.SCANVOXELSIZE");
    std::cout << "scan_voxel_size : " << scan_voxel_size << std::endl;
    voxel_size_step = tag_setting.get<float>("DATATYPE.VOXELSIZESTEP");
    std::cout << "voxel_size_step : " << voxel_size_step << std::endl;
    min_voxel_size = tag_setting.get<float>("DATATYPE.MINVOXELSIZE");
    std::cout << "min_voxel_size : " << min_voxel_size << std::endl;
    max_voxel_size = tag_setting.get<float>("DATATYPE.MAXVOXELSIZE");
    std::cout << "max_voxel_size : " << max_voxel_size << std::endl;
    min_pointsize = tag_setting.get<float>("DATATYPE.MINPOINTSIZE");
    std::cout << "min_pointsize : " << min_pointsize << std::endl;
    max_pointsize = tag_setting.get<float>("DATATYPE.MAXPOINTSIZE");
    std::cout << "max_pointsize : " << max_pointsize << std::endl;
    std::cout << "#[DATATYPE] config end #" << std::endl;
    std::cout << " " << std::endl;

    std::cout << "#[SAVE] config start #" << std::endl;
    save_single_size = tag_setting.get<int>("SAVE.SAVESINGLESIZE");
    std::cout << "save_single_size : " << save_single_size << std::endl;
    tile_size = tag_setting.get<float>("SAVE.TILESIZE");
    std::cout << "tile_size : " << tile_size << std::endl;
    grid_size = tag_setting.get<float>("SAVE.GRIDSIZE");
    std::cout << "grid_size : " << grid_size << std::endl;
    map_voxel_size = tag_setting.get<float>("SAVE.MAPVOXELSIZE");
    std::cout << "map_voxel_size : " << map_voxel_size << std::endl;
    use_map_filter = tag_setting.get<float>("SAVE.USEMAPFILTER");
    std::cout << "use_map_filter : " << use_map_filter << std::endl;
    use_ground_filter = tag_setting.get<float>("SAVE.USEGROUNDFILTER");
    std::cout << "use_ground_filter : " << use_ground_filter << std::endl;
    save_debug_data = tag_setting.get<bool>("SAVE.SAVEDEBGUDATA");
    std::cout << "save_debug_data : " << save_debug_data << std::endl;
    use_cut_save_map = tag_setting.get<bool>("SAVE.USECUTSAVETYPE");
    std::cout << "use_cut_save_map : " << use_cut_save_map << std::endl;
    save_sub_start_id = tag_setting.get<int>("SAVE.SUBSAVESARTID");
    std::cout << "save_sub_start_id : " << save_sub_start_id << std::endl;
    save_sub_end_id = tag_setting.get<int>("SAVE.SUBSAVEENDID");
    std::cout << "save_sub_end_id : " << save_sub_end_id << std::endl;
    std::cout << "#[SAVE] config end #" << std::endl;
    std::cout << " " << std::endl;

    std::cout << "#[MAPPER] config start #" << std::endl;
    use_gps = tag_setting.get<bool>("MAPPER.USEGPS");
    std::cout << "use_gps : " << use_gps << std::endl;
    use_loop_detect = tag_setting.get<bool>("MAPPER.USELOOP");
    std::cout << "use_loop_detect : " << use_loop_detect << std::endl;
    use_manual_lc = tag_setting.get<bool>("MAPPER.USEMANUALLC");
    std::cout << "use_manual_lc : " << use_manual_lc << std::endl;
    odom_type = tag_setting.get<int>("MAPPER.ODOMTYPE");
    std::cout << "odomtype : " << odom_type << std::endl;
    search_map_num = tag_setting.get<int>("MAPPER.SEARCHMAPNUM");
    std::cout << "search_map_num : " << search_map_num << std::endl;
    kf_dis = tag_setting.get<float>("MAPPER.KFDISSELECT");
    std::cout << "kf_dis : " << kf_dis << std::endl;
    loop_match_tpye = tag_setting.get<int>("MAPPER.LOOPMATCHTPYE");
    std::cout << "loop_match_tpye : " << loop_match_tpye << std::endl;
    search_loop_radius = tag_setting.get<int>("MAPPER.SEARCHLOOPRAD");
    std::cout << "search_loop_radius : " << search_loop_radius << std::endl;
    lc_score = tag_setting.get<float>("MAPPER.LCSCORE");
    std::cout << "lc_score : " << lc_score << std::endl;
    std::cout << "#[MAPPER] config start #" << std::endl;
    std::cout << " " << std::endl;
    match_type = tag_setting.get<string>("Register.MATCHTYPE");
    std::cout << "match type : " << match_type << std::endl;
    bsubmap = tag_setting.get<bool>("Register.SUBMAP");
    std::cout << "SUBMAP : " << bsubmap << std::endl;
}



#endif
