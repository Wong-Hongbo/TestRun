/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     Commonfig.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:06:43
*/
#ifndef COMMONFIG_HH
#define COMMONFIG_HH
#include <pcl/common/common.h>
#include <iostream>
#include <chrono>

using namespace std;
const static double DEG2RAD =  3.1415926/18000.0f;
typedef Eigen::Matrix< double , 4 , 1> Matrix41d;
typedef Eigen::Matrix< double, 4 , 4> Matrix44d;
typedef Eigen::Matrix< double , 2 , 1> Matrix21d;
typedef Eigen::Matrix< double, 3 , 1> Matrix31d;
typedef Eigen::Matrix< double, 3 , 3> Matrix33d;
#define   BASE_X      (19695588)
#define   BASE_Y      (3125798)
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    int type; //0:无效 1:有效 2: 失效
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time)(int, type, type))

struct PointXYZIRPYTGPS
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    float lat_dev;
    float lon_dev;
    float height_dev;
    double lon;
    double lat;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYTGPS,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, roll, roll)
                                  (float, pitch, pitch)
                                  (float, yaw, yaw)
                                  (double, time, time)
                                  (float, lat_dev, lat_dev)
                                  (float, lon_dev, lon_dev)
                                  (float, height_dev, height_dev)
                                  (double, lon,lon )
                                  (double, lat,lat) )
struct Pointlooppose
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    int loop_from;
    int loop_to;
    int type;//0：短边 1：长边
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(Pointlooppose, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(int, loop_from, loop_from)(int, loop_to, loop_to)(int, type, type))

struct PointXYZIN
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    int num;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIN, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, num, num))


struct PointXYZIL
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
//  uint16_t ring;                      ///< laser ring number
  int label;                     ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (int, label, label))

struct PointXYZRGBIL
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  PCL_ADD_RGB;
  float    intensity;                 ///< laser intensity reading
//  uint16_t ring;                      ///< laser ring number
  int label;                     ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBIL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, b ,b)
                                  (uint8_t, g ,g)
                                  (uint8_t, r ,r)
                                  (float, intensity, intensity)
                                  (int, label, label))

using PointTypeSeg = pcl::PointXYZRGBL;
using PointType3D = pcl::PointXYZI;
using PointType6D = PointXYZIRPYT;
using PointTGPS =  PointXYZIRPYTGPS;
using PointType2D = pcl::PointXY;
using PointT = pcl::PointXYZI;

struct RPYpose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double time;
    double lat_dev;
    double lon_dev;
    double height_dev;
    double lon;
    double lat;
};

struct gnssBack
{
    RPYpose previous_gnss_pose;
    double previous_gnss_time;
};

struct eur_angle
{
    double roll;
    double pitch;
    double yaw;
};

struct localizer_state
{
    string Good_State;
    string localizer_lost;
    string Network_Bad;
};

struct Index2d
{
    int x;
    int y;
};

struct LoadMapIndex
{
   Index2d tileID;
   std::string tilefilename;
};
struct LoadMapIndexNew
{
    int inx;
    int iny;
    pcl::PointCloud<pcl::PointXYZ> tile_Map;
    std::string tilefilename;
};
struct PointWithDistance
{
    PointType3D point;
    float distance;
};

struct PoseStamped
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double time;
};

struct linear
{
    float x;
    float y;
    float z;  // m/s
};

struct angular
{
    float x;
    float y;
    float z;  //   rad/s
};

struct ImuStamped
{
    linear linear_acceleration;
    angular  angular_velocity;
    double time;
};

struct Vstate
{
    double velocity;
    double time;
};

struct VehicleStated
{
    Vstate state;
};

struct MapProp
{
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    int bigmap_width;
    int bigmap_length;
    int littlemap_diameter;
    int totalmap_width;
    int totalmap_length;
    int littlemap_number_x;  // 一个大地图中小地图
    int littlemap_number_y;  // 一个大地图中小地图
};

struct mapIndex
{
    double xlx;
    double ylx;
    double xrs;
    double yrs;
    int idx;   //col
    int idy;   //row
    string filename; //pcd
    pcl::PointCloud<pcl::PointXYZI>  cloud;
    bool bExistFile;
};


#endif
