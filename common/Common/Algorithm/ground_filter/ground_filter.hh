/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     ground_filter.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-12-06 01:07:01
*/
#ifndef GROUND_FILTER_HH
#define GROUND_FILTER_HH

#include <Common/Commonfig.hh>
#include <Common/ParamServer.hh>

inline bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b){
    return a.z<b.z;
}

class GROUNDFILTER
{
public:
    GROUNDFILTER();
    void ExtractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>& p_sorted);
    void EstimatePlane(void);
    void GroundFilterRun(pcl::PointCloud<pcl::PointXYZI> laserCloudIn);

    pcl::PointCloud<pcl::PointXYZI>::Ptr g_seeds_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_ground_pc;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_not_ground_pc;
    pcl::PointCloud<PointXYZIL>::Ptr g_all_pc;
    Eigen::MatrixXf normal_;
    float th_dist_d_;

    float ground_d_;
};
#endif
