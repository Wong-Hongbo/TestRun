#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <signal.h>
#include <stdio.h>
#include <chrono>
#include <unistd.h>
#include <iostream>

#include "pcl/common/common.h"
#include "pcl/io/pcd_io.h"
#include <pcl/common/transforms.h>  

#include <Eigen/Core>

#include "common/type.h"
#include "map/p_map.h"
#include "map/map.h"

typedef Eigen::Vector3d Posed;
typedef Eigen::Vector3f Posef;

struct KeyFrame
{
    Posef pose;
    PCLPoints point;
    KeyFrame() {}
    KeyFrame(Posef pose_, PCLPoints point_){
        pose = pose_;
        point = point_;
    }
};

// 创建概率地图
P_map create_Map(std::vector<KeyFrame> keyframe);

// 关键帧匹配到概率地图
Posef match(PCLPoints& point, Posef init_pose, P_map& p_map);

#endif // MAPPER2D_H
