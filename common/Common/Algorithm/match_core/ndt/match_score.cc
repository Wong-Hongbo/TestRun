#include "match_score.hh"
#include <pcl/point_types.h>
//给定 费米 参数 kt\ mu
MatchingScore::MatchingScore()
    : fermi_kT_(0.05)
    , fermi_mu_(0.25),
      tree_ptr_(new pcl::search::KdTree<pcl::PointXYZI>)
{

}

void MatchingScore::SetInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr)
{
    static size_t points_size = 0;
    if (points_size != pointcloud_ptr->points.size())
    {
        tree_ptr_->setInputCloud(pointcloud_ptr);
        points_size = pointcloud_ptr->points.size();
    }
}

// 这个地方采用KdTree
void MatchingScore::SetSearchMethodTarget(pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_ptr)
{
    tree_ptr_ = tree_ptr;
}

//计算匹配得分
float MatchingScore::CalcMatchingScore(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr)
{
    point_with_distance_array_ = ConvertPointWithDistance(pointcloud_ptr);
    const double s0 = CalcFermiDistributionFunction(0, fermi_kT_, fermi_mu_);  //to normalize to 1 可认为0,1之间
    double score_sum = 0;
    for(const auto point_with_distance : point_with_distance_array_)
    {
        const double s = CalcFermiDistributionFunction(point_with_distance.distance, fermi_kT_, fermi_mu_);
        score_sum += s / s0;
    }
    const double score = point_with_distance_array_.empty()
                     ? 0
                     : score_sum / point_with_distance_array_.size();
    return score;
}

//通过曲线拟合可得，当前可近似费米分布 f(E) = {1/[exp((E－Ef)/kT) + 1]}
float MatchingScore::CalcFermiDistributionFunction(const float x, const float kT, const float mu)
{
    return 1.0 / (std::exp((x-mu)/kT)+1.0);
}

//将点转换到距离
std::vector< PointWithDistance> MatchingScore::ConvertPointWithDistance(pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr)
{
    std::vector< PointWithDistance> point_with_distance_array;
    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);
    for (const auto& point : pointcloud_ptr->points)
    {
        PointWithDistance point_with_distance;
        //搜索半径 为1m
        tree_ptr_->nearestKSearch(point, 1, nn_indices, nn_dists);
        point_with_distance.point = point;
        point_with_distance.distance = std::sqrt(nn_dists[0]);
        point_with_distance_array.push_back(point_with_distance);
    }
    return point_with_distance_array;
}

double MatchingScore::ProcessMatchScore(pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr)
{
    SetInputTarget(map_ptr);
//    updateMatchingScore(points_ptr);
    return CalcMatchingScore(points_ptr);
}
