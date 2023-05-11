/*
 * @Description: 转换p_map到cv_map,两个地图的区别是cv_map的坐标系在左下角图片原点，p_map不是
 * @Author: zhong haixing
 * @Date: 2021-01-20 15:31:31
 * @Email: zhonghaixing01@countrygarden.com.cn
 * @FilePath: /loop_detect/inc/loop_detect/cv_map.h
 */

#ifndef CV_MAP_H
#define CV_MAP_H

#include <iostream>
#define HAVE_OPENCV_HIGHGUI
#include <opencv2/opencv.hpp>
//#include <glog/logging.h>
#include <Eigen/Core>

//概率地图
#include "p_map.h"

class CV_map
{
private:
    cv::Mat cv_map;

public:
    CV_map(/* args */);
    ~CV_map();

    float map_scale;
    int map_width, map_height;
    float map_ori_x, map_ori_y;

    bool get_map_flag = false;
    cv::Mat cv_map_show;

    void copy_form_p_map(P_map& probability_map_);
    void inserd_pose(Eigen::Matrix4f& robot_pose_);

    void show_img();
    cv::Mat get_cv_map(){
        return cv_map;
    }
};



#endif

