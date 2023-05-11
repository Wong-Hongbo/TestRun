#ifndef BIG_SLIDINGWINDOW_C_H
#define BIG_SLIDINGWINDOW_C_H

#include "stdlib.h"
#include "vector"
#include "MLadarDatan.hh"
#include "HDLadarDatan.hh"
#include "PositiveObstaclen.hh"
#include "opencv2/opencv.hpp"

struct PointGrayValue
{
    double x;
    double y;
    UINT8   gray;
};

class BIG_POSITIVEOBSTACLE_MSG
{
public:

    //Constructor
    BIG_POSITIVEOBSTACLE_MSG();
    ~BIG_POSITIVEOBSTACLE_MSG();
    std::vector<PointGrayValue> big_GrayScalePoints;    // 存储xy相对坐标,以及gray值
    std::vector<PointGrayValue> big_GrayScaleObPoints;    // 存储xy相对坐标,以及gray值
    std::vector<cv::Point3i> big_PosObPoints;   // 有效障碍物点像素坐标, xy像素值，z高度值
    UINT8 *big_PositiveObstacle;   // 存储0,1代表有效
    UINT8 *big_GrayScaleMap;    // 存储灰度值
    UINT8 *big_Tr_GrayScaleMap;
};

class big_slidingwindow_c
{
public:
    big_slidingwindow_c();
    ~big_slidingwindow_c();

//    void setInputCloud(float *_M_data, int num);
    void filter_measurement();

    PointCoordinate64 inputclouds[64][PACKETNUM*6];

//    float *M_data;
//    int M_num;

//    int *obstacle_map;

//    int *num_obstacle_map;       // 30m in left,right and back side, 75m in the front


    int *min_obstacle_map;
    int *max_obstacle_map;

    int *min_height_above_150;
    int *max_height_below_150;

    unsigned char *tmp_pos_img;

    void compute_obstacle_map_from_points(std::vector<cv::Point3i> points, BIG_POSITIVEOBSTACLE_MSG * _PositiveOb_map, bool display = false);
    void compute_obstacle_map(BIG_POSITIVEOBSTACLE_MSG * _PositiveOb_map, bool display = false);
    void compute_grayscale_map(BIG_POSITIVEOBSTACLE_MSG * _GrayScale_map, bool display = false);
    void compute_transformed_grayscale_map(float delta_x,float delta_y,float delta_a,double last_azimuth,BIG_POSITIVEOBSTACLE_MSG * _GrayScale_map);

    void setInputCloud(HDLADARDATA_MSG *HDLadarData);
    void setInputCloud(MLADARDATA_MSG *MLadarData);
    void setInputCloud(PointCoordinate64 points[64][2100]);
    void displayPosMap(BIG_POSITIVEOBSTACLE_MSG * _PositiveOb_map);
    void displayGrayScaleMap(BIG_POSITIVEOBSTACLE_MSG * _GrayScale_map);

    bool process_32_data;
    bool process_64_data;

private:
    double scale;
    int valid_lidar_distance;
    double resolution;
    int img_cols;
    int img_rows;
    int img_size;
    std::vector<cv::Point3i> input_points_;

};


#endif // BIG_SLIDINGWINDOW_C_H
