#ifndef _BUILD_MAP_H_
#define _BUILD_MAP_H_
#include <opencv2/opencv.hpp>
#include "config/point.h"
#include "config/transform.h"
#include "config/util.h"
//#include "gridlinetraversal.h"

using namespace  std;

class BuildMap2{
public:
    BuildMap2(string data_path);
    ~BuildMap2();

    void setOrigin(cv::Vec2f origin);
    void worldToNineMap(const cv::Point2i pos_w, cv::Point2i &pos_m);
    void worldToTotalMap(const cv::Point2i pos_w, cv::Point2i &pos_m);

    void computeMissPoints(const std::vector<cv::Vec2f> hits, std::vector<cv::Vec2f> &miss);
    void computeNineMapOrigin(const cv::Point3i gauss_pos);
    void readNineMaps();
    void savePreNineMaps();
    void saveTotalMap();
    int updateMap(cv::Point3i gauss_pos,
                  const cv::Vec3d rpy,
                  const std::vector<cv::Point3i> hits_points,
                  const std::vector<cv::Point3i> obstal_points);

    int updateMap(cv::Point3i gauss_pos,
                  const cv::Vec3d rpy,
                  const std::vector<cv::Point3i> ground_points,
                  const std::vector<cv::Point3i> nonground_points,
                  const std::vector<cv::Point3i> obstal_points);

    int updateTotalMap(cv::Point3i gauss_pos,
                       const cv::Vec3d rpy,
                       const std::vector<cv::Point3i> ground_points,
                       const std::vector<cv::Point3i> nonground_points,
                       const std::vector<cv::Point3i> obstal_points);

    void getRobotMap(cv::Mat &map);
    void getProbMap(cv::Mat &map);

private:
    bool setGridBel(const int &x, const int &y, const double& bel);
    bool getGridBel ( const int& x, const int& y, double& bel);
    bool setGridLogBel(const int &x, const int &y, const double& log_bel);
    bool getGridLogBel(const int &x, const int &y, double& log_bel);
//    void showMap(std::string map_name, cv::Mat img, cv::Point2i robot_pose);

    void writeXml(std::string filename, cv::Mat mat);
    void readXml(std::string filename, cv::Mat &mat);

    void writeBin(std::string filename, cv::Mat mat);
    void readBin(std::string filename, cv::Mat &mat);
    void savePng(std::string filename, cv::Mat mat);
    void printfMat(const cv::Mat & mat);

    void getLittleMapOrigins(std::vector<cv::Point2i> &origins);
    void combineLittleMaps(std::vector<cv::Point2i> origins, cv::Mat &map);
    void OnHeightMouse(int event, int x, int y);
    static void OnHeightMouse(int event, int x, int y, int, void* userdata);

private:
    std::vector<cv::Point2i> hits_;
    std::vector<cv::Point2i> miss_;
    int resolution_;
    cv::Vec2f origin_;

    // little map
    int littlemap_index_x_;
    int littlemap_index_y_;
    cv::Point2i ninemap_origin_;
    cv::Point2i littlemap_origin_[3][3];

    int littlemap_size_;

    // big map
    int bigmap_index_x_;
    int bigmap_index_y_;
    cv::Point2i bigmap_origin_;
    int bigmap_size_;

    int littlemap_rows_;
    int littlemap_cols_;
    cv::Mat nine_maps_;
    cv::Mat nine_height_maps_;
    cv::Mat bel_maps_;
    cv::Mat total_map_;
    cv::Mat little_map_[3][3];
    cv::Mat show_map_;
    cv::Point2i current_map_center_;

    cv::Point2i total_map_origin_min_;
    cv::Point2i total_map_origin_max_;

    xs::Transform tf_;

    // allocate this large array only once

    double p_free_;
    double p_occ_;
    double p_prior_;
    double log_free_;
    double log_occ_;
    double log_prior_;
    double log_min_;
    double log_max_;

    cv::Point2i robot_pose_m_;
    std::string littlemap_save_path_;
    bool first_update_;

    cv::Point2d map_range_min_;
    cv::Point2d map_range_max_;
    int base_x_;
    int base_y_;
    std::string range_txt_path_;

    int total_map_cols_;
    int total_map_rows_;
    cv::Mat last_obstcal_map_;
    cv::Mat totalmap_bel_;

    int robot_radius_;
    string data_path;
};

#endif
