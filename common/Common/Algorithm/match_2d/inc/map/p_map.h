#ifndef P_MAP_H
#define P_MAP_H

#include <eigen3/Eigen/Core>
#include <vector>
#include <deque>
#include <algorithm>

#include "probability_values.h"
//#include "compress/compress.h"
#include "common/type.h"

struct Lidar_range
{
    Lidar_range(Eigen::Vector3f min_xy_, Eigen::Vector3f max_xy_):min_xy(min_xy_), max_xy(max_xy_){};
    Lidar_range():min_xy(Eigen::Vector3f::Zero()), max_xy(Eigen::Vector3f::Zero()){};
    ~Lidar_range(){};
    Eigen::Vector3f min_xy;
    Eigen::Vector3f max_xy;

    //1   2
    //4   3
    std::vector<Eigen::Vector3f> vertex_points_;

    void calculate_vertex(){
        vertex_points_.resize(4);

        Eigen::Vector3f point_;
        point_[0] = min_xy[0];
        point_[1] = max_xy[1];
        point_[2] = 0.;
        vertex_points_[0] = point_;

        point_[0] = max_xy[0];
        point_[1] = max_xy[1];
        point_[2] = 0.;
        vertex_points_[1] = point_;

        point_[0] = max_xy[0];
        point_[1] = min_xy[1];
        point_[2] = 0.;
        vertex_points_[2] = point_;

        point_[0] = min_xy[0];
        point_[1] = min_xy[1];
        point_[2] = 0.;
        vertex_points_[3] = point_;
    }

    void tra_vertex(const Eigen::Matrix4f& tra) {
        for(uint i = 0; i < vertex_points_.size(); i++){
            vertex_points_[i] = tra.block<3, 3>(0, 0) * vertex_points_[i] + Eigen::Vector3f(tra(0, 3), tra(1, 3), tra(2, 3));
        }
    }
};

//定义概率地图
class P_map
{
private:
    //更新栅格的下标
    std::vector<int> update_indexs;
public:
    P_map(/* args */){ }
    ~P_map(){ release_memory(); }

    void release_memory(){
        p_map.resize(0);
        p_map_cells.resize(0);
        cells_.resize(0);
    }

    //param
    //概率地图的宽高
    float p_map_origin_x = 0., p_map_origin_y = 0., p_map_origin_z = 0.;
    float p_map_quan_x = 0., p_map_quan_y = 0., p_map_quan_z = 0., p_map_quan_w = 1.;
    float p_map_resolution = 0.05;
    int p_map_width = 0, p_map_height = 0;

    //概率地图
    std::vector<uint16> p_map;
    //记录栅格概率地图的hit与visit次数
    std::vector<P_map_cell> p_map_cells;
    
    // Probabilites mapped to 0 to 255.
    std::vector<uint8> cells_;
    int cells_origin_size_ = 0;

    Eigen::Array2i offset_;

    int size(){
        return p_map.size() * sizeof(uint16) + 
               p_map_cells.size() * sizeof(P_map_cell) +
               cells_.size() * sizeof(uint8);
    }

    //从map坐标系转到OpenCV map坐标系（原点在左下角）
    Eigen::Matrix4d To_opencv_axit(Eigen::Matrix4d robot_pose_){
        int x = (robot_pose_(0, 3) - p_map_origin_x) / p_map_resolution;
        int y = (robot_pose_(1, 3) - p_map_origin_y) / p_map_resolution;

        robot_pose_(0, 3) = x;
        robot_pose_(1, 3) = y;

        return robot_pose_;
    }

    //从世界坐标系转换到栅格坐标系
    Eigen::Array2i ConvertWorld2GridIndex(float x, float y) const {
        Eigen::Array2i index;
        index[0] = std::ceil((x - p_map_origin_x) / p_map_resolution);
        index[1] = std::ceil((y - p_map_origin_y) / p_map_resolution);
        return index;
    }

    //判断index是否有效
    bool isValidGridIndex(Eigen::Array2i index) const {
        if(index[0] >= 0 && index[0] < p_map_width && index[1] >= 0 && index[1] < p_map_height)
            return true;
        return false;
    }

    //转到栅格坐标系转到cell index
    int ToFlatIndex(const Eigen::Array2i& cell_index) const {
        int flat_index = p_map_width * cell_index[1] + cell_index[0];
        if(flat_index >= p_map_width * p_map_height){
//            LOG_FIRST_N(INFO, 10) << "地图溢出！！" ;
            return 0;
        }
//        LOG_IF(INFO, flat_index < 0) << "cell_index : " << cell_index;
//        CHECK_GE(flat_index, 0);
        return flat_index;
    }

    //取对应栅格的概率
    double GetProbabilityFormIndex(Eigen::Array2i index) const {
        return 1.0 - ValueToCorrespondenceCost(p_map[ToFlatIndex(index)]);
    }

    // Sets the probability of the cell at 'cell_index' to the given
    // 'probability'. Only allowed if the cell was unknown before.
    //概率地图对应位置概率设定
    void SetProbability(const Eigen::Array2i& cell_index, const float probability) {
        p_map[ToFlatIndex(cell_index)] = BoundedFloatToValue(probability, 0.1, 0.9);
    }

    //概率值转换为对应的value值（0-32767）
    uint16 BoundedFloatToValue( const float float_value,
                                const float lower_bound,
                                const float upper_bound) const {
        const int value = RoundToInt((Clamp(float_value, lower_bound, upper_bound) - lower_bound) *
                                     (32766.f / (upper_bound - lower_bound))) + 1;
        // DCHECK for performance.
//        DCHECK_GE(value, 1);
//        DCHECK_LE(value, 32767);
        return value;
    }

    void ResetByRange(Lidar_range& lidar_range_){
        p_map.clear();
        p_map_cells.clear();
        std::vector<uint16>().swap(p_map);
        std::vector<P_map_cell>().swap(p_map_cells);

        p_map_origin_x = lidar_range_.min_xy[0];
        p_map_origin_y = lidar_range_.min_xy[1];

        p_map_width = int((lidar_range_.max_xy[0] - lidar_range_.min_xy[0])/p_map_resolution);
        p_map_height = int((lidar_range_.max_xy[1] - lidar_range_.min_xy[1])/p_map_resolution);

        // p_map.resize(p_map_width * p_map_height);
        std::vector<uint16> new_cells(p_map_width * p_map_height, 0);
        P_map_cell p_cell;
        std::vector<P_map_cell> new_p_cells(p_map_width * p_map_height, p_cell);
        p_map = new_cells;
        p_map_cells = new_p_cells;
    }

    void CopyFormOldmap(P_map& old_map_){
//        CHECK(old_map_.p_map_resolution == p_map_resolution);

        int old_width = old_map_.p_map_width;
        int old_height = old_map_.p_map_height;

        int offset = int((old_map_.p_map_origin_x - p_map_origin_x) / p_map_resolution) + 
                     int((old_map_.p_map_origin_y - p_map_origin_y) / p_map_resolution) * p_map_width;

//        CHECK(old_width <= p_map_width && old_height <= p_map_height) << "new map size < old map size";

        for(int i = 0; i < old_height; i++){
            for(int j = 0; j < old_width; j++){
                int index = offset + j + i * p_map_width;
                if(index < 0 || index > p_map_width * p_map_height){
                    MAPPER_WARN("copy out of range");
                    continue;
                }
//                CHECK(index >= 0) << index;
//                CHECK(index <= p_map_width * p_map_height) << index << " - " << p_map_width * p_map_height << " - " << i << " - " << j;
                p_map[offset + j + i * p_map_width] = old_map_.p_map[j + i * old_width];
                p_map_cells[offset + j + i * p_map_width] = old_map_.p_map_cells[j + i * old_width];
            }
        }
    }

    int GetCellValue(const Eigen::Array2i& xy_index) const {
        const Eigen::Array2i local_xy_index = xy_index - offset_;
        // The static_cast<unsigned> is for performance to check with 2 comparisons
        // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
        // local_xy_index.x() >= wide_limits_.num_x_cells ||
        // local_xy_index.y() >= wide_limits_.num_y_cells
        // instead of using 4 comparisons.
        if (!isValidGridIndex(local_xy_index)) {
            return 255;
        }
        return cells_[local_xy_index.x() + local_xy_index.y() * p_map_width];
    }

    // Maps values from [0, 255] to [min_score, max_score].
    float ToCellScore(float value) const {
        float min_score_ = 1.f - 0.1;
        float max_score_ = 1.f - 0.9;

        return min_score_ + value * ((max_score_ - min_score_) / 255.f);
    }

    void GrawMapSize(bool p_map_flag = true){

        if(p_map_flag == true){
            std::vector<uint16> old_map_cells = p_map;

            int old_width = p_map_width;
            int old_height = p_map_height;

            p_map_width = old_width * 2;
            p_map_height = old_height * 2;

            std::vector<uint16> new_cells(p_map_width * p_map_height, 0);

            p_map_origin_x = p_map_origin_x - old_width / 2.f * p_map_resolution;
            p_map_origin_y = p_map_origin_y - old_height / 2.f * p_map_resolution;

            int offset = old_width / 2 + old_height / 2 * p_map_width;

            for(int i = 0; i < old_height; i++){
                for(int j = 0; j < old_width; j++){
//                    int index = offset + j + i * p_map_width;
//                    CHECK(index >= 0) << index;
//                    CHECK(index <= p_map_width * p_map_height) << index << " - " << p_map_width * p_map_height << " - " << i << " - " << j;
                    new_cells[offset + j + i * p_map_width] = old_map_cells[j + i * old_width];
                }
            }

            p_map.clear();
            std::vector<uint16>().swap(p_map);
            p_map = new_cells;
        }
        else{
            std::vector<uint8> old_map_cells = cells_;

            int old_width = p_map_width;
            int old_height = p_map_height;

            p_map_width = old_width * 2;
            p_map_height = old_height * 2;

            std::vector<uint8> new_cells(p_map_width * p_map_height, 0);

            p_map_origin_x = p_map_origin_x - old_width / 2.f * p_map_resolution;
            p_map_origin_y = p_map_origin_y - old_height / 2.f * p_map_resolution;

            int offset = old_width / 2 + old_height / 2 * p_map_width;

            for(int i = 0; i < old_height; i++){
                for(int j = 0; j < old_width; j++){
//                    int index = offset + j + i * p_map_width;
//                    CHECK(index >= 0) << index;
//                    CHECK(index <= p_map_width * p_map_height) << index << " - " << p_map_width * p_map_height << " - " << i << " - " << j;
                    new_cells[offset + j + i * p_map_width] = old_map_cells[j + i * old_width];
                }
            }

            cells_.clear();
            std::vector<uint8>().swap(cells_);
        }
        
    }

    void rotation_90(){

        //替换width和height
        int p_map_width_rota_ = p_map_height;
        int p_map_height_rota_ = p_map_width;

        //概率地图
        std::vector<uint16> p_map_rota_ = p_map;
        //记录栅格概率地图的hit与visit次数
        std::vector<P_map_cell> p_map_cells_rota_ = p_map_cells;

        for(int i = 0; i < p_map_height_rota_; i++){
            for(int j = 0; j < p_map_width_rota_; j++){
                p_map_rota_[i * p_map_width_rota_ + j] = p_map[(p_map_height - j - 1) * p_map_width + i];
                p_map_cells_rota_[i * p_map_width_rota_ + j] = p_map_cells[(p_map_height - j - 1) * p_map_width + i];
            }
        }

        p_map_height = p_map_height_rota_;
        p_map_width = p_map_width_rota_;
        p_map = p_map_rota_;
        p_map_cells = p_map_cells_rota_;

    }

    void addscan(Point_cloud2d& scan, std::vector<uint16>& hit_table_, std::vector<uint16>& miss_table_);
    bool map_index_update(const Eigen::Array2i& cell_index, const std::vector<uint16>& table);

};



#endif
