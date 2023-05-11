#ifndef MATCHING_SCORE_H
#define MATCHING_SCORE_H

#include <pcl/search/kdtree.h>
//#include <Commonfig.hh>
#include <Common/Commonfig.hh>

class MatchingScore
{
public:
    MatchingScore();
    void SetInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr);
    void SetSearchMethodTarget(pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_ptr);
    float CalcMatchingScore(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr);

    double ProcessMatchScore(pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr);
    void SetFermikT(const float fermi_kT)
    {
        fermi_kT_ = fermi_kT;
    }

    float GetFermikT()
    {
        return fermi_kT_;
    }

    void SetFermiMu(const float fermi_mu)
    {
        fermi_mu_ = fermi_mu;
    }

    float GetFermiMu()
    {
        return fermi_mu_;
    }

    std::vector< PointWithDistance > GetPointWithDistanceArray()
    {
        return point_with_distance_array_;
    }

//private:
    std::vector< PointWithDistance > ConvertPointWithDistance(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr);
    float CalcFermiDistributionFunction(const float x, const float kT, const float mu);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_ptr_;  // kd tree
    std::vector< PointWithDistance > point_with_distance_array_;

    float fermi_kT_;
    float fermi_mu_;
    MatchingScore* matchingscore;
};

#endif

