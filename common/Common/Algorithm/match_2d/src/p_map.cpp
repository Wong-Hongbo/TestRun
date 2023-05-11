/*
 * @Description: 栅格概率地图
 * @Author: zhong haixing
 * @Date: 2020-08-13 16:19:06
 * @Email: zhonghaixing01@countrygarden.com.cn
 * @FilePath: /loop_detect/src/p_map.cpp
 */
#include "map/p_map.h"

/**
 * @description: 画线算法
 * @param {*}
 * @return {*}
 */
std::vector<Eigen::Array2i> TraceLine(int x0, int y0, int x1, int y1){

    Eigen::Array2i tmpIndex;
    std::vector<Eigen::Array2i> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1) {
        ystep = 1;
    }
    else {
        ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            pointX = y;
            pointY = x;
        }
        else {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX) {
            y += ystep;
            error -= deltaX;
        }
        if(pointX == x1 && pointY == y1) continue;

        //保存所有的点
        tmpIndex << pointX, pointY;
        gridIndexVector.push_back(tmpIndex);
    }
    return gridIndexVector;
}

/**
 * @description: 栅格概率更新
 * @param {*}
 * @return {*}
 */
bool P_map::map_index_update(const Eigen::Array2i& cell_index, const std::vector<uint16>& table){
    const int flat_index = ToFlatIndex(cell_index);

    uint16* table_index = &p_map[flat_index];

    //如果已经更新过了就不更新
    if(*table_index >= kUpdateMarker){
        return false;
    }
    //这一步操作是拿现在p_map内的value作为table的index去取值，这样更新更快
    // pro_map.p_map[flat_index] = table[pro_map.p_map[flat_index]];

    p_map[flat_index] = table[*table_index];

    update_indexs.push_back(flat_index);
    return true;
}

/**
 * @description: 获取需要插值的scan点，只有当scan两个点之间的角度大于阈值角度时，进行插值
 * @param {Point_cloud2d} &scan
 * @return {Point_cloud2d} 需要插值的scan点
 */
Point_cloud2d get_interpolation_scan(Point_cloud2d &scan){


    Point_cloud2d ori_scan;
    Point_cloud2d interpolation_scan;
    //将点云转到lidar坐标系
    for(auto point : scan.points){
        Eigen::Vector2f point_ori = Eigen::Vector2f(point[0] - scan.robot_pose[0], point[1] - scan.robot_pose[1]);
        ori_scan.points.push_back(Eigen::Rotation2Df(-scan.robot_pose[2]) * point_ori);
    }

    int point_size = scan.points.size();
    //计算激光角分辨率-(这里使用倍加福的激光分辨率，要是使用其他激光应该也可以的，只是绘制的分辨率而已)
    double incri_theta = 0.00436332309619;

    //遍历每个点
    for(int i = 0; i < point_size - 1 ; i++){
        double theta_0 = atan2(ori_scan.points[i][1], ori_scan.points[i][0]);
        double theta_1 = atan2(ori_scan.points[i+1][1], ori_scan.points[i+1][0]);
        double delta_theta = fabs(theta_1 - theta_0);

        //计算i与i+1点之间的delta theta  如果两个点之间的角度大于设定值5度，这里选用固定角度是因为可能使用不同分辨率的激光，
        //这样会导致分辨率低的激光角度会很大，  当delta theta 大于5度 则进行插值
        if (delta_theta > 4.5 / 180.0 * M_PI){
            //计算两个点之间需要插值多少个激光点
            int insert_points_num = round(delta_theta / incri_theta);
            double start_theta = std::min(theta_0, theta_1);
            for(int j = 1; j <= insert_points_num; j++){
                Eigen::Vector2f insert_point = Eigen::Vector2f(0.3 * cos(start_theta + incri_theta * j), 0.3 * sin(start_theta + incri_theta * j));
                Eigen::Vector2f rota_point = Eigen::Rotation2Df(scan.robot_pose[2]) * insert_point;
                interpolation_scan.points.push_back(Eigen::Vector2f(rota_point[0] + scan.robot_pose[0], rota_point[1] + scan.robot_pose[1]));
            }
        }
    }

    interpolation_scan.robot_pose = scan.robot_pose;

    return interpolation_scan;
}

/**
 * @description: 向栅格地图中添加scan帧
 * @param {*}
 * @return {*}
 */
void P_map::addscan(Point_cloud2d& scan, std::vector<uint16>& hit_table_, std::vector<uint16>& miss_table_){
    Eigen::Vector3f origin_robot_pose = scan.robot_pose;

    //先更新占据点
    for(size_t data_index = 0; data_index < scan.points.size(); data_index++){

        Eigen::Array2i end_point_index = ConvertWorld2GridIndex(scan.points[data_index][0], scan.points[data_index][1]);

        if (isValidGridIndex(end_point_index)) {
            //更新占据概率
            map_index_update(end_point_index, hit_table_);
            p_map_cells[ToFlatIndex(end_point_index)].hit_num += 4;
            p_map_cells[ToFlatIndex(end_point_index)].visit += 4;
        }
    }

    //取每个激光点对应的画线原点
    Eigen::Array2i origin_point_index = ConvertWorld2GridIndex(origin_robot_pose[0],
                                                               origin_robot_pose[1]);
                                            
    //再更新空闲点，以免占据点被抹去
    for(size_t data_index = 0; data_index < scan.points.size(); data_index++){

        Eigen::Array2i end_point_index = ConvertWorld2GridIndex(scan.points[data_index][0], scan.points[data_index][1]);

        if (isValidGridIndex(end_point_index)) {
            //利用画线算法找到到激光点之间的栅格坐标
            std::vector<Eigen::Array2i> free_index = TraceLine( origin_point_index[0], origin_point_index[1],
                                                                end_point_index[0], end_point_index[1]);
            for (size_t j = 0; j < free_index.size(); j++){
                //更新空闲概率
                map_index_update(free_index[j], miss_table_);
                p_map_cells[ToFlatIndex(free_index[j])].visit += 4;
            }
        }
    }    

    //当已有地图的时候进行地图更新
    while(!update_indexs.empty()){
        //概率地图去掉更新marker
        p_map[update_indexs.back()] -= kUpdateMarker;
        
        //弹出该点
        update_indexs.pop_back();
    }
}

/**
 * @description: 概率转换到地图实际存储的值
 * @param {*}
 * @return {*}
 */
uint8 ComputeCellValue(const float probability) {
    float min_score_ = 1.f - 0.1;
    float max_score_ = 1.f - 0.9;

  const int cell_value = RoundToInt(
      (probability - min_score_) * (255.f / (max_score_ - min_score_)));
//  CHECK_GE(cell_value, 0);
//  CHECK_LE(cell_value, 255);
  return cell_value;
}


