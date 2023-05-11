/*
 * @Description: 类型定义
 * @Author: zhong haixing
 * @Date: 2020-12-02 10:33:38
 * @Email: zhonghaixing01@countrygarden.com.cn
 * @FilePath: /loop_detect/src/type.cpp
 */
#include "common/type.h"

/**
 * @description: 3d的变换矩阵转换成2d的位姿向量（x、y、theta）
 * @param {Eigen::Matrix4}
 * @return {std::array<T, 3>}
 */
std::array<float, 3> matrix_to_array(const Eigen::Matrix4f& pose_3d){
    std::array<float, 3> pose_2d = {pose_3d(0, 3),
                                    pose_3d(1, 3),
                                    getYawEulerAngle(Eigen::Quaternionf(pose_3d.block<3, 3>(0, 0)))};
    return pose_2d;
}

/**
 * @description: 3d的变换矩阵转换成2d的位姿向量（x、y、theta）
 * @param {Eigen::Matrix4}
 * @return {std::array<T, 3>}
 */
std::array<double, 3> matrix_to_arrayd(const Eigen::Matrix4f& pose_3d){
    std::array<double, 3> pose_2d = {pose_3d(0, 3),
                                     pose_3d(1, 3),
                                     getYawEulerAngle(Eigen::Quaternionf(pose_3d.block<3, 3>(0, 0)))};
    return pose_2d;
}

/**
 * @description: 3d的位姿向量（x、y、z yaw pitch row）转换成3d的变换矩阵
 * @param {std::array<float, 6>}
 * @return {Eigen::Matrix4f}
 */
Eigen::Matrix4f array_to_matrix_3d(std::array<float, 6> pose_3d) {
    Eigen::Matrix4f matrix_3d = Eigen::Matrix4f::Identity();

    matrix_3d(0, 3) = pose_3d[0];
    matrix_3d(1, 3) = pose_3d[1];
    matrix_3d(2, 3) = pose_3d[2];

    Eigen::Matrix3f rota_matrix;
    rota_matrix = Eigen::AngleAxisf(pose_3d[3], Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(pose_3d[4], Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(pose_3d[5], Eigen::Vector3f::UnitX());
    matrix_3d.block<3, 3>(0, 0) = rota_matrix;

    return matrix_3d;
}

/**
 * @description: 3d的变换矩阵 转换成 3d的位姿向量（x、y、z yaw pitch row）
 * @param {Eigen::Matrix4f}
 * @return {std::array<float, 6>}
 */
std::array<float, 6> matrix_to_array_3d(Eigen::Matrix4f pose_3d) {
    std::array<float, 6> array_3d;
    array_3d[0] = pose_3d(0, 3);
    array_3d[1] = pose_3d(1, 3);
    array_3d[2] = pose_3d(2, 3);

    Eigen::Vector3f rota_vec = pose_3d.block<3, 3>(0, 0).matrix().eulerAngles(2, 1, 0);
    array_3d[3] = rota_vec(0);
    array_3d[4] = rota_vec(1);
    array_3d[5] = rota_vec(2);

    return array_3d;
}

/**
 * @description: 从四元数中得到yaw
 * @param {Eigen::Quaternionf}
 * @return {float}
 */
float getYawEulerAngle(const Eigen::Quaternionf& q) {
    // yaw (z-axis rotation)
    float siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    float cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    float yaw = atan2(siny_cosp, cosy_cosp);
    return yaw;
}

/**
 * @description: pcl点云转换到分支定界匹配的点云格式
 * @param {pcl点云格式}
 * @return {Point_cloud2d格式}
 */
Point_cloud2d PclToPointcloud2d(pcl::PointCloud<pcl::PointXYZI> cloud_, Eigen::Matrix4f& scan_pose_){
    Point_cloud2d return_scan;
    for(size_t i = 0; i < cloud_.points.size(); i++){
        return_scan.points.push_back(Eigen::Vector2f(cloud_.points[i].x, cloud_.points[i].y));
    }
    return_scan.robot_pose = Eigen::Vector3f(scan_pose_(0, 3),
                                             scan_pose_(1, 3),
                                             getYawEulerAngle(Eigen::Quaternionf(scan_pose_.block<3, 3>(0, 0))));
    return return_scan;
}

/**
 * @description: 转换到Point_cloud2d点云格式
 * @param {*}
 * @return {*}
 */
Point_cloud2d PclToPointcloud2d(PCLPoints& pcl_cloud){
    Point_cloud2d out_cloud;
    for(auto value : pcl_cloud.points){
        out_cloud.points.push_back(Eigen::Vector2f(value.x, value.y));
    }
    return out_cloud;
}


/**
 * @description: Point_cloud2d点云格式转换到pcl点云格式
 * @param {*}
 * @return {*}
 */
PCLPoints Pointcloud2dToPcl(Point_cloud2d& cloud_2d){
    PCLPoints out_cloud;
    for(auto point : cloud_2d.points){
        PCLPOINT p1;
        p1.x = point.x();
        p1.y = point.y();
        p1.z = 0.;
        out_cloud.points.push_back(p1);
    }
    return out_cloud;
}
