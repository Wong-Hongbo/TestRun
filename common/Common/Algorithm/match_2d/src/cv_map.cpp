/*
 * @Description: 转换p_map到cv_map,两个地图的区别是cv_map的坐标系在左下角图片原点，p_map不是
 * @Author: zhong haixing
 * @Date: 2021-01-20 15:29:38
 * @Email: zhonghaixing01@countrygarden.com.cn
 * @FilePath: /loop_detect/src/cv_map.cpp
 */

#include "map/cv_map.h"

CV_map::CV_map(/* args */)
{
//    cv::namedWindow("img", CV_WINDOW_NORMAL);
//    cv::resizeWindow("img", 800, 800);
}

CV_map::~CV_map()
{
}

void CV_map::copy_form_p_map(P_map& probability_map_){

    map_scale = probability_map_.p_map_resolution;
    map_width = probability_map_.p_map_width;
    map_height = probability_map_.p_map_height;
    map_ori_x = probability_map_.p_map_origin_x;
    map_ori_y = probability_map_.p_map_origin_y;

    cv::Mat img(map_height, map_width, CV_8U);

    for(int y = 0; y < map_height; y++) {
        for(int x = 0; x < map_width; x++) {
            int i = x + (map_height - y - 1) * map_width;
            
            auto value = probability_map_.p_map_cells[i];
            if(value.visit == -1){
//                img.data[x + y * map_width] = 0;
                img.data[x + y * map_width] = 125;
                continue;
            }
            float cell_p = (float)value.hit_num/(float)value.visit;
//            img.data[x + y * map_width] = cell_p * 254 + 1;
            if(cell_p > 0.25){
                img.data[x + y * map_width] = 000;
            }
            else{
                img.data[x + y * map_width] = 254;
            }
        }
    }

    cv_map_show = img;
    get_map_flag = true;
}

void CV_map::inserd_pose(Eigen::Matrix4f& robot_pose_){
    if(!get_map_flag) return;

    int x = (robot_pose_(0, 3) - map_ori_x) / map_scale;
    int y = (robot_pose_(1, 3) - map_ori_y) / map_scale;

    auto point = cv::Point2d(x, map_height - y);
    cv::cvtColor(cv_map_show, cv_map_show, CV_GRAY2BGR);
    cv_map_show = cv_map;
    cv::circle(cv_map_show, point, 10, cv::Scalar(0, 255, 0), -1);
}

void CV_map::show_img(){

    if(!get_map_flag) return;

    if(cv_map.rows < 10 || cv_map.cols < 10) {
        return;
    }
    
    cv::imshow("img", cv_map);
    cv::waitKey(1);
}


