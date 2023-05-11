/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName   ProbabilityGridMap
* @fileName      probability_map.h
* @brief
* @author        fuxiang
* @date          2019-11-14
*/
#ifndef PROBABILITY_MAP_H
#define PROBABILITY_MAP_H
#include "big_slidingwindow_c.h"
#include "slam/config/pose2d.h"
#include "build_map.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "HDLadarDatan.hh"

namespace xs {



typedef struct{
    Pose2d robot_pose;
    std::vector<cv::Point3i> points;
}RawData;

// Calculates 'sqrt(x*x + y*y + z*z)'.
template <typename T>
constexpr double Dist3d(T x, T y, T z) {
  return std::sqrt((x)*(x) + (y)*(y) + (z)*(z));
}

template <typename T>
constexpr double Dist2d(T x, T y) {
  return std::sqrt((x)*(x) + (y)*(y));
}

typedef struct{
    double min_range;
    double max_range;
    bool groundPtExist;
    bool nongroundPtExist;
    cv::Point3i min_nonground_pt;        // 最近非地面点
    cv::Point3i max_ground_pt;           // 最远地面点
}ScanData;

class ProbabilityMap{
public:
    ProbabilityMap(string data_path);
    ~ProbabilityMap();

    void GetMap(cv::Mat &map);

    void UpdateMap(const cv::Point3i gauss_pos,
                   const cv::Vec3d rpy,
                   const std::vector<cv::Point3i> full_points,
                   const std::vector<cv::Point3i> scan_points);

    void UpdateMap(const cv::Point3i gauss_pos,
                   const cv::Vec3d rpy,
                   const std::vector<cv::Point3i> full_points);

    void UpdateMap(const cv::Point3i gauss_pos,
                   const cv::Vec3d rpy,
                   std::string file_name);

    void GetProbMap(cv::Mat &map);


private:
    void LoadLidarWithPoints(const std::string filename, std::vector<cv::Point3i> &all_ground_points,
                   std::vector<cv::Point3i> &all_unground_points);

    bool LoadLidarWithScan(const std::string filename, std::vector<cv::Point3i> &all_ground_points,
                   std::vector<cv::Point3i> &all_unground_points);

    void PointsToScan(const std::vector<cv::Point3i> &points, std::vector<cv::Point3i> &singlering_points);

    void PointsFilter(const std::vector<cv::Point3i> &points, std::vector<cv::Point3i> &all_ground_points,
                      std::vector<cv::Point3i> &all_unground_points);

    void ShowScan(std::string map_name, const std::vector<cv::Point3i> &points);

    void ShowScan(std::string map_name, const std::vector<cv::Point3d> &points);

    void OnMouse(int event, int x, int y);
    static void OnMouse(int event, int x, int y, int, void* userdata);

private:

    int range_min_;
    int range_max_;
    double angle_increment_;
    int grid_size_;
    int scan_num_;


    BuildMap* build_map_;
    big_slidingwindow_c * big_slidingwindow_;

    PointCoordinate64 inputclouds[64][PACKETNUM*6];
    double pre_ground_z = 0;
    float ground_z = 0;
    cv::Mat nonground_mat_;
};
}

#endif // PROBABILITY_MAP_H
