/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     ground_filter.cc
* @brief
* @author       Yu Hui Liang
* @date         2020-12-06 01:07:13
*/
#include "ground_filter.hh"


GROUNDFILTER::GROUNDFILTER()
{
    g_seeds_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    g_ground_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    g_not_ground_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    g_all_pc.reset(new pcl::PointCloud<PointXYZIL>);
}

void GROUNDFILTER::ExtractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>& p_sorted)
{
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for(int i=0;i<p_sorted.points.size() && cnt<3;i++){
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt!=0?sum/cnt:0;// in case divide by 0
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for(int i=0;i<p_sorted.points.size();i++){
        if(p_sorted.points[i].z < lpr_height + 3.0){
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}
void GROUNDFILTER::EstimatePlane(void)
{
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    ground_d_ = -(normal_.transpose()*seeds_mean)(0,0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = 0.3 - ground_d_;

    // return the equation parameters
}
void GROUNDFILTER::GroundFilterRun(pcl::PointCloud<pcl::PointXYZI> laserCloudIn)
{
    g_ground_pc->clear();
    g_not_ground_pc->clear();
    g_all_pc->clear();

    auto start = std::chrono::system_clock::now();

    // 1.Msg to pointcloud
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn_org ;
    laserCloudIn_org.clear();
    laserCloudIn_org += laserCloudIn;
    // For mark ground points and hold all points
    PointXYZIL point;
    for(size_t i=0;i<laserCloudIn.points.size();i++){
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        //        point.ring = 0;
        point.label = 0u;// 0 means uncluster
        g_all_pc->points.push_back(point);
    }
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
    // 2.对高度进行排序
    std::sort(laserCloudIn.points.begin(),laserCloudIn.end(),point_cmp);

    // 3.剔除错误的点
    // As there are some error mirror reflection under the ground,
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<pcl::PointXYZI>::iterator it = laserCloudIn.points.begin();
    for(int i=0;i<laserCloudIn.points.size();i++){
        if(laserCloudIn.points[i].z < -0.1*0.8){
            it++;
        }else{
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(),it);
    // 4. 提取种子地面
    ExtractInitialSeeds(laserCloudIn);
    g_ground_pc = g_seeds_pc;

    // 5. 地面平面提取
    //    for(int i=0;i<num_iter_;i++)
    {
        EstimatePlane();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(laserCloudIn_org.points.size(),3);
        int j =0;
        for(auto p:laserCloudIn_org.points){
            points.row(j++)<<p.x,p.y,p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points*normal_;
        // threshold filter
        for(int r=0;r<result.rows();r++){
            if(result[r]<th_dist_d_){
                g_all_pc->points[r].label = 1u;// means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }else{
                g_all_pc->points[r].label = 0u;// means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }

    auto end = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0;
    //std::cout << "Time loop : " << diff <<  " ms " <<std::endl;

    //    viewer();
    //    g_all_pc->clear();
}
