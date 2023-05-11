//#######################################################################################
//#  This source code is about a ground filtering algorithm for airborn LiDAR data      #
//#  based on physical process simulations, specifically cloth simulation.              #
//#                                     Copyright 2021                                  #
//#                  Changsha Xingshen Intelligent Technology Co., Ltd                  #
//#                               (http://www.xingshentech.com/)                        #
//#                                                                                     #
//#                                  Authored By Meng Deyuan                            #
//#                                                                                     #
//#######################################################################################

#ifndef OBSTACLEMAP_H
#define OBSTACLEMAP_H
// 系统库
#include <iostream>
#include <vector>
#include <stack>
#include <deque>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <pcl/io/pcd_io.h>
// 自身库
#include "particle.h"
#include "delabella.h"

#define MINZ -2.0f
#define SEARCH_DIS 144.0f
#define FILTER_RES 0.2f
#define ATAN_LIST_NUM 6000
#define RADIAN2ANGLE 57.29578f
#define ANGLE2RADIAN 0.01745f
#define CLOTH_Z_COMP 0.03f
#define MIN_MOVE_DIS 0.005f


struct CarModelParams
{
    float boundary_rear;
    float boundary_front;
    float boundary_left;
    float boundary_right;
    float suspended_height;
    float ground_height;
    CarModelParams():boundary_rear(-1.0),
                     boundary_front(1.0),
                     boundary_left(-1.0),
                     boundary_right(1.0),
                     suspended_height(2.0),
                     ground_height(0.0){}
};


struct PolarGridmapParams
{
    float radis_range;
    int angle_range_min;
    int angle_range_max;
    float radis_resolution;
    int angle_resolution;
    float max_search_dis;
    float slope_thresh;
    float height_thresh;
    float height_increase;
    float time_step;
    int rigidness;
    int iterations;
    int noise_num;
    int num_thread;
    PolarGridmapParams():radis_range(60.0),
                         angle_range_min(0),
                         angle_range_max(360),
                         radis_resolution(0.5),
                         angle_resolution(2),
                         max_search_dis(20.0),
                         slope_thresh(15.0),
                         height_thresh(0.10),
                         height_increase(0.0025),
                         time_step(0.6),
                         rigidness(2),
                         iterations(30),
                         noise_num(6){}
};

// type--0:地面点，1：地面区域正障碍点, 2：负障碍点，3：无关点（噪点、超高点）
struct PointCloudT
{
    float x;
    float y;
    float z;
    float intensity;
    int pos_a;
    int pos_r;
    int type;
    int filter_a;
    int filter_r;
    int filter_z;
    int filter_idx;
};


struct LinePoint
{
    float r;
    float z;
    LinePoint():r(0.0f), z(0.0f){}
    LinePoint(float temp_r, float temp_z) { r = temp_r; z = temp_z; }
};


class ObstacleMap
{
public:
    ObstacleMap(CarModelParams car_params,
                PolarGridmapParams polar_params);
    void ShowOutput();
    /* 建立极坐标栅格地图，过滤点云得到障碍点
     */
    void ObstalePointFilter();
    /* 预处理主雷达数据，得到对应的极坐标
     */
    void PointCloudPreprocess(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_data);
public:
    // 动态更新数据
    std::vector<PointCloudT> m_pointclouds;
    int m_valid_num;
private:
    /* 计算三角面片的法向量夹角
     */
    float CalculateTriangleSlope(const std::vector<PointCloudT> &pc,
                                 const DelaBella_Triangle *dela,
                                 float max_length);
    /* 计算角度值
     */
    int CountAtanAngle(float x, float y);
    /* 查表得atan角度值, 输入x应大于等于0
     */
    int GetAtanAngleFromList(int x);
    /* 点云投影到地形图，寻找每个栅格的对应点云
     */
    void RasterTerrain(float max_h);
    /* 地形构建迭代主循环
     */
    void TimeStep();
    /* 地形对比函数，如果粒子比地面低则置粒子为不可移动
     */
    void TerrainCollision();
    /* 地形平滑函数
     */
    void TerrainSmooth();
    /* 使用估计的地形来更新点属性
     */
    void UpdatePointTypeByGround();
    /* 极坐标栅格地图表示：用于障碍点提取
     * 正后方为0度，顺时针方向角度增大
     * 包括盲区区域、限制区域
     */
    void InitPolarGridmapParams();
    void AddNeighbor(int curr_a, int curr_r, int neighbor_a, int neighbor_r, bool round_flag);
private:
    const CarModelParams m_kCarModelParams;
    const PolarGridmapParams m_kPolarGridmapParams;
    int m_radis_max_idx;
    int m_angle_max_idx;
    std::vector<int> m_atan_angle;
    std::vector<float> m_radis_position;
    std::vector<int> m_blind_area;
    std::vector<std::vector<Particle> > m_cloth_terrain;
};

#endif // OBSTACLEMAP_H
