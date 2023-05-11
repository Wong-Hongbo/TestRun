#include <QDir>

#include "build_map.h"
#include "Common.h"
#include "config/line.h"
#include "opencv2/opencv.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>

BuildMap2::BuildMap2(string env_data_path)
{

    data_path = env_data_path;
//    std::string ugv_ini = "/data/ugv.ini";
//    boost::property_tree::ptree pt;
//    boost::property_tree::ini_parser::read_ini(ugv_ini, pt);
//    std::string station = pt.get<std::string>("ugvinfo.station", "");

    range_txt_path_ = data_path + "/range.txt";

    std::cout << "range_txt_path_ : " << range_txt_path_ << std::endl;

    std::string line;
    std::ifstream input(range_txt_path_.c_str());
    std::getline(input, line);
    std::vector<std::string> strv;
    boost::split(strv, line, boost::is_any_of(","));
    std::vector<int> range_value;
    for (size_t i = 0; i < strv.size(); i++)
    {
        boost::trim(strv[i]);

        std::vector<std::string> valuev;
        boost::split(valuev, strv[i], boost::is_any_of("="));
        boost::trim(valuev[1]);

        range_value.push_back(std::atoi(valuev[1].c_str()));
    }

    int range_min_x = range_value[0];
    int range_min_y = range_value[1];
    int range_max_x = range_value[2];
    int range_max_y = range_value[3];
//    resolution_ = range_value[4];
    resolution_ = 10;
    base_x_ = range_value[5];
    base_y_ = range_value[6];
    LogInfo("minx = %d, miny = %d, maxx = %d, maxy = %d, GridSize = %d\n", (int)map_range_min_.x,
            (int)map_range_min_.y, (int)map_range_max_.x, (int)map_range_max_.y, resolution_);

    total_map_rows_ = (map_range_max_.y - map_range_min_.y)/resolution_;
    total_map_cols_ = (map_range_max_.x - map_range_min_.x)/resolution_;

    littlemap_size_ = 3000;
    bigmap_size_ = 100000;

    littlemap_cols_ = littlemap_size_ / resolution_;
    littlemap_rows_ = littlemap_cols_;
    nine_maps_ = cv::Mat(cv::Size(3*littlemap_rows_, 3*littlemap_cols_), CV_64FC1, 0.5);

    nine_height_maps_ = cv::Mat::zeros(3*littlemap_rows_, 3*littlemap_cols_, CV_8UC1);
    last_obstcal_map_ = cv::Mat::zeros(3*littlemap_rows_, 3*littlemap_cols_, CV_8UC1);

    current_map_center_.x = 0;
    current_map_center_.y = 0;

    p_free_ = 0.4;
    p_occ_ = 0.6;
    p_prior_ = 0.5;

    log_free_ = log(p_free_/(1.0-p_free_));
    log_occ_ = log(p_occ_/(1.0-p_occ_));
    log_prior_ = log(p_prior_/(1.0 - p_prior_));

    littlemap_save_path_ = data_path + "/littlemap";
    if (access(littlemap_save_path_.c_str(), 0) == -1)
    {
        if(mkdir(littlemap_save_path_.c_str(), 0744) == -1)
        {
            printf("cannot create directory[%s] failed\n", littlemap_save_path_.c_str());
        }
    }
    else
    {
//        std::string cmd = "rm -rf " + littlemap_save_path_ + "/*";
//        printf("cmd = %s\n", cmd.c_str());
//        system(cmd.c_str());
        QDir dir(QString::fromStdString(littlemap_save_path_));
        dir.removeRecursively();
    }

    first_update_ = true;
    robot_radius_ = 50; // cm
}

BuildMap2::~BuildMap2()
{

}

void BuildMap2::setOrigin(cv::Vec2f origin)
{
    origin_ = origin;
}

/*
 *map(0,0)
 * .    ^y
 *      |
 *      |
 * ---------->x
 *      |
 *      |
 * . // ninemap_origin
 *
*/
void BuildMap2::worldToNineMap(const cv::Point2i pos_w, cv::Point2i &pos_m)
{
    cv::Point2i offset;
    offset.x = pos_w.x - ninemap_origin_.x;
    offset.y = pos_w.y - ninemap_origin_.y;
    pos_m.x = offset.x/resolution_;
    pos_m.y = nine_maps_.rows - offset.y/resolution_;
}

void BuildMap2::worldToTotalMap(const cv::Point2i pos_w, cv::Point2i &pos_m)
{
    cv::Point2i offset;
    offset.x = pos_w.x - map_range_min_.x;
    offset.y = pos_w.y - map_range_min_.y;
    pos_m.x = offset.x/resolution_;
    pos_m.y = total_map_rows_ - offset.y/resolution_;
}

void BuildMap2::computeMissPoints(const std::vector<cv::Vec2f> hits, std::vector<cv::Vec2f> &miss)
{

}

void BuildMap2::computeNineMapOrigin(const cv::Point3i gauss_pos)
{
    cv::Point2i center_littlemap_origin;
    if (gauss_pos.x >= 0 && gauss_pos.y >= 0)
    {
        center_littlemap_origin.x = gauss_pos.x - gauss_pos.x%littlemap_size_;
        center_littlemap_origin.y = gauss_pos.y - gauss_pos.y%littlemap_size_;
    }
    else if (gauss_pos.x >= 0 && gauss_pos.y < 0)
    {
        center_littlemap_origin.x = gauss_pos.x - gauss_pos.x%littlemap_size_;
        center_littlemap_origin.y = gauss_pos.y + (-gauss_pos.y)%littlemap_size_ - littlemap_size_ ;
    }
    else if (gauss_pos.x < 0 && gauss_pos.y >= 0)
    {
        center_littlemap_origin.x = gauss_pos.x + (-gauss_pos.x)%littlemap_size_ - littlemap_size_;
        center_littlemap_origin.y = gauss_pos.y - gauss_pos.y%littlemap_size_;
    }
    else if (gauss_pos.x < 0 && gauss_pos.y < 0)
    {
        center_littlemap_origin.x = gauss_pos.x + (-gauss_pos.x)%littlemap_size_ - littlemap_size_;
        center_littlemap_origin.y = gauss_pos.y + (-gauss_pos.y)%littlemap_size_ - littlemap_size_ ;
    }

    current_map_center_.x = center_littlemap_origin.x + littlemap_size_/2;
    current_map_center_.y = center_littlemap_origin.y + littlemap_size_/2;

    ninemap_origin_.x = center_littlemap_origin.x - littlemap_size_;
    ninemap_origin_.y = center_littlemap_origin.y - littlemap_size_;

    // calculate littlemap origin xy
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            littlemap_origin_[i][j].x = ninemap_origin_.x + j*littlemap_size_;
            littlemap_origin_[i][j].y = ninemap_origin_.y + i*littlemap_size_;
        }
    }
//    LogInfo("nine map_center (%d, %d) ninemap_origin(%d, %d)\n", current_map_center_.x, current_map_center_.y,
//            ninemap_origin_.x, ninemap_origin_.y);
}

void BuildMap2::readNineMaps()
{
    std::string map_name;
    int roi_x;
    int roi_y;
    std::string reso_str;
    if (resolution_ == 10)
        reso_str = "10cm";
    else if (resolution_ == 20)
    {
        reso_str = "20cm";
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            map_name = littlemap_save_path_ + "/XS_" + std::to_string(littlemap_origin_[i][j].x) + "_" +
                    std::to_string(littlemap_origin_[i][j].y) + ".bin";
//            std::cout << "read Nine Maps = " << map_name << std::endl;


            cv::Mat tmp_img;

            readBin(map_name, tmp_img);
//            readXml(map_name, tmp_img);

            roi_x = j*littlemap_cols_;
            roi_y = (2-i)*littlemap_rows_;
//            printf("roi_x = %d, roi_y = %d\n", roi_x, roi_y);
            // little maps combine to ninemap
            cv::Mat roi_img = nine_maps_(cv::Rect(roi_x, roi_y, littlemap_rows_, littlemap_cols_));
            tmp_img.copyTo(roi_img);


            // read nine height map
            std::string height_name = littlemap_save_path_ + "/XS_" + std::to_string(littlemap_origin_[i][j].x) + "_" +
                    std::to_string(littlemap_origin_[i][j].y) + ".png";
            cv::Mat height_img = cv::imread(height_name.c_str(), -1);
            if (height_img.empty() == 1)
            {
                height_img = cv::Mat::zeros(littlemap_rows_, littlemap_cols_, CV_8UC1);
            }
            cv::Mat height_roi_img = nine_height_maps_(cv::Rect(roi_x, roi_y, littlemap_rows_, littlemap_cols_));
            height_img.copyTo(height_roi_img);
        }
    }
//    cv::imwrite("nine_maps.png", nine_maps_);

}

void BuildMap2::savePreNineMaps()
{
    std::string map_name;
    std::string png_name;
    cv::Mat little_map;
    int roi_x;
    int roi_y;
    std::string reso_str;
    if (resolution_ == 10)
        reso_str = "10cm";
    else if (resolution_ == 20)
    {
        reso_str = "20cm";
    }
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            map_name = littlemap_save_path_ + "/XS_" + std::to_string(littlemap_origin_[i][j].x) + "_" +
                    std::to_string(littlemap_origin_[i][j].y) + ".bin";
            roi_x = j*littlemap_cols_;
            roi_y = (2-i)*littlemap_rows_;
            little_map = nine_maps_(cv::Rect(roi_x, roi_y, littlemap_rows_, littlemap_cols_)).clone();
            writeBin(map_name, little_map);

            png_name = littlemap_save_path_ + "/XS_" + std::to_string(littlemap_origin_[i][j].x) + "_" +
                    std::to_string(littlemap_origin_[i][j].y) + ".png";
//            std::cout << "png:name " << png_name << std::endl;


            cv::Mat little_height_map = nine_height_maps_(cv::Rect(roi_x, roi_y, littlemap_rows_, littlemap_cols_)).clone();
            cv::imwrite(png_name, little_height_map);
//            savePng(png_name, little_height_map);
//            writeXml(map_name, little_map);
//            showMap("little_map", little_map);
        }
    }

}

void BuildMap2::saveTotalMap()
{

}

int BuildMap2::updateMap(cv::Point3i gauss_pos,
                        const cv::Vec3d rpy,
                        const std::vector<cv::Point3i> hits_points,
                        const std::vector<cv::Point3i> obstal_points)
{
    gauss_pos.x += base_x_;
    gauss_pos.y += base_y_;
//    LogInfo("gauss_pos(%d, %d, %d) rpy(%d, %d, %d), hits_points.size = %d\n",
//            gauss_pos.x, gauss_pos.y, gauss_pos.z, rpy[0], rpy[1], rpy[2], hits_points.size());
    int distance = std::max(abs(gauss_pos.x - current_map_center_.x), abs(gauss_pos.y - current_map_center_.y));
    if (first_update_)
    {
        computeNineMapOrigin(gauss_pos);
        readNineMaps();
        computeNineMapOrigin(gauss_pos);
        first_update_ = false;
        return 0;
    }

    if (distance > littlemap_size_/2)
    {
        // save the pre nine maps
        savePreNineMaps();
        saveTotalMap();
        // calculate 9 little map origin
        computeNineMapOrigin(gauss_pos);
        readNineMaps();
    }

    // trans gauss pos to ninemap
    cv::Point2i gauss_pos_w;
    cv::Point2i gauss_pos_m;
    gauss_pos_w.x = gauss_pos.x;
    gauss_pos_w.y = gauss_pos.y;
    worldToNineMap(gauss_pos_w, gauss_pos_m);

    robot_pose_m_ = gauss_pos_m;

    double R[3][3];          // rotation matrix
    tf_.eulerAnglesToRotationMatrix(rpy, R);


    cv::Point2i hits_points_tmp_w;
    cv::Point2i hits_points_tmp_m;



    cv::Mat ninemap_hits_count = cv::Mat::zeros(nine_maps_.rows, nine_maps_.cols, CV_8UC3);

    uint8_t key = -1;
    cv::Point3i lidar_in_robot;
    lidar_in_robot.x = 0;
    lidar_in_robot.y = 71;
    lidar_in_robot.z = 118;
    cv::Point2i lidar_w;
    lidar_w.x = lidar_in_robot.x * R[0][0]
            + lidar_in_robot.y * R[0][1]
            + lidar_in_robot.z * R[0][2]
            + gauss_pos.x;
    lidar_w.y = lidar_in_robot.x * R[1][0]
            + lidar_in_robot.y * R[1][1]
            + lidar_in_robot.z * R[1][2]
            + gauss_pos.y;

    cv::Point2i lidar_m;
    worldToNineMap(lidar_w, lidar_m);

//    ninemap_hits_count.at<cv::Vec3b>(lidar_m) = cv::Vec3b(0, 255, 0);
//    ninemap_hits_count.at<cv::Vec3b>(gauss_pos_m) = cv::Vec3b(0, 0, 255);

//    cv::circle(ninemap_hits_count, lidar_m, 10, cv::Scalar(0, 255, 0), 3);
//    cv::circle(ninemap_hits_count, gauss_pos_m, 10, cv::Scalar(0, 0, 255), 3);
    // trans hits points to ninemap
    for (size_t i = 0; i < hits_points.size(); i++)
    {
        // trans hits points to global world
        hits_points_tmp_w.x = hits_points[i].x * R[0][0]
                + hits_points[i].y * R[0][1]
                + hits_points[i].z * R[0][2]
                + gauss_pos.x;
        hits_points_tmp_w.y = hits_points[i].x * R[1][0]
                + hits_points[i].y * R[1][1]
                + hits_points[i].z * R[1][2]
                + gauss_pos.y;

        worldToNineMap(hits_points_tmp_w, hits_points_tmp_m);

        ninemap_hits_count.at<cv::Vec3b>(hits_points_tmp_m) = cv::Vec3b(128,128,128);

        std::vector<cv::Point2i> line;
        double logbel;
        double bel;
        LineBresenham(lidar_m, hits_points_tmp_m, line);
        if (line.size() <=1 )
            continue;

        double dist = Dist2d(hits_points_tmp_m.y - lidar_m.y, hits_points_tmp_m.x - lidar_m.x)*resolution_;

        if (dist < 300)
        {
            for (size_t j = line.size()-2; j > 0; j--)
            {
                getGridBel(line[j].y, line[j].x, bel);
                if (bel == p_prior_)
                {
                    logbel = log_prior_ + log_free_;
                    setGridLogBel(line[j].y, line[j].x, logbel);
                    continue;
                }

                getGridLogBel(line[j].y, line[j].x, logbel);

                double dist_tmp = Dist2d(line[j].y - lidar_m.y, line[j].x - lidar_m.x)*resolution_;
                if (dist_tmp < 50 && logbel > log_free_)
                    break;

                logbel += log_free_;
    //            printf("logbel = %f\n", logbel);
                setGridLogBel(line[j].y, line[j].x, logbel);
            }
        }
        else
        {
            // miss
            for (size_t j = line.size()-2; j > 0; j--)
            {
                getGridLogBel(line[j].y, line[j].x, logbel);
                double dist_tmp = Dist2d(line[j].y - lidar_m.y, line[j].x - lidar_m.x)*resolution_;
                if (logbel > log_occ_ && dist_tmp < 300)
                    break;

                logbel += log_free_;
                //            printf("logbel = %f\n", logbel);
                setGridLogBel(line[j].y, line[j].x, logbel);
            }
        }

        // hits
        getGridLogBel(hits_points_tmp_m.y, hits_points_tmp_m.x, logbel);
        logbel += log_occ_;
        setGridLogBel(hits_points_tmp_m.y, hits_points_tmp_m.x, logbel);
    }

//    ninemap_hits_count.at<cv::Vec3b>(cv::Point(nine_maps_.rows/2, nine_maps_.cols/2)) = cv::Vec3b(0, 0, 255);
//    cv::Point lidar_m;
//    lidar_m.x = nine_maps_.cols/2;
//    lidar_m.y = nine_maps_.rows/2;

#if 1
    cv::Point2i obstal_points_tmp_w;
    cv::Point2i obstal_points_tmp_m;
    for (size_t i = 0; i < obstal_points.size(); i++)
    {

        // trans hits points to global world
        obstal_points_tmp_w.x = obstal_points[i].x * R[0][0]
                + obstal_points[i].y * R[0][1]
                + gauss_pos.x;
        obstal_points_tmp_w.y = obstal_points[i].x * R[1][0]
                + obstal_points[i].y * R[1][1]
                + gauss_pos.y;

        worldToNineMap(obstal_points_tmp_w, obstal_points_tmp_m);

        double logbel;
        // obstal_points
        getGridLogBel(obstal_points_tmp_m.y, obstal_points_tmp_m.x, logbel);
        logbel += log_occ_;
        setGridLogBel(obstal_points_tmp_m.y, obstal_points_tmp_m.x, logbel);
    }
#endif

//    showMap("nine_map", nine_maps_, lidar_m);
//    cv::namedWindow("count_map", CV_NORMAL);
//    cv::imshow("count_map", ninemap_hits_count);

    return 0;

}

int BuildMap2::updateMap(cv::Point3i gauss_pos,
                        const cv::Vec3d rpy,
                        const std::vector<cv::Point3i> ground_points,
                        const std::vector<cv::Point3i> nonground_points,
                        const std::vector<cv::Point3i> obstal_points)
{
    gauss_pos.x += base_x_;
    gauss_pos.y += base_y_;

    int distance = std::max(abs(gauss_pos.x - current_map_center_.x), abs(gauss_pos.y - current_map_center_.y));
    if (first_update_)
    {
        computeNineMapOrigin(gauss_pos);
        readNineMaps();
        computeNineMapOrigin(gauss_pos);
        first_update_ = false;
        return 0;
    }

    if (distance > littlemap_size_/2)
    {
        // save the pre nine maps
        savePreNineMaps();
        saveTotalMap();
        // calculate 9 little map origin
        computeNineMapOrigin(gauss_pos);
        readNineMaps();
    }

    // trans gauss pos to ninemap
    cv::Point2i gauss_pos_w;
    cv::Point2i gauss_pos_m;
    gauss_pos_w.x = gauss_pos.x;
    gauss_pos_w.y = gauss_pos.y;
    worldToNineMap(gauss_pos_w, gauss_pos_m);

    robot_pose_m_ = gauss_pos_m;

    double R[3][3];          // rotation matrix
    tf_.eulerAnglesToRotationMatrix(rpy, R);

    // 更新obstal points
    cv::Point3i lidar_in_robot;
    lidar_in_robot.x = 0;
    lidar_in_robot.y = 90;
    lidar_in_robot.z = 83;
    cv::Point2i lidar_w;
    cv::Point2i lidar_m;
    lidar_w.x = lidar_in_robot.x * R[0][0]
            + lidar_in_robot.y * R[0][1]
            + lidar_in_robot.z * R[0][2]
            + gauss_pos.x;
    lidar_w.y = lidar_in_robot.x * R[1][0]
            + lidar_in_robot.y * R[1][1]
            + lidar_in_robot.z * R[1][2]
            + gauss_pos.y;

    worldToNineMap(lidar_w, lidar_m);

    cv::Point2i obstal_points_w;
    cv::Point2i obstal_points_m;
    cv::Mat matObstacleFlag= cv::Mat::zeros(nine_maps_.rows, nine_maps_.cols, CV_8UC1);


    for (size_t i = 0; i < obstal_points.size(); i++)
    {
        obstal_points_w.x = obstal_points[i].x * R[0][0]
                + obstal_points[i].y * R[0][1]
                + gauss_pos.x;
        obstal_points_w.y = obstal_points[i].x * R[1][0]
                + obstal_points[i].y * R[1][1]
                + gauss_pos.y;

        worldToNineMap(obstal_points_w, obstal_points_m);

        matObstacleFlag.at<uchar>(obstal_points_m)=255;

        double logbel;

        getGridLogBel(obstal_points_m.y, obstal_points_m.x, logbel);
        if(obstal_points[i].z<=20 && nine_height_maps_.at<uchar>(obstal_points_m)<=20)
            logbel += log_occ_;
        else
            logbel += log_occ_;
        setGridLogBel(obstal_points_m.y, obstal_points_m.x, logbel);

        if (nine_height_maps_.at<uchar>(obstal_points_m) < obstal_points[i].z)
        {
            nine_height_maps_.at<uchar>(obstal_points_m) = obstal_points[i].z;
        }

//        if (obstal_points[i].z < 10)
//            totalmap_obstcal_height_.at<uchar>(obstal_points_m) = 1;
//        else
//            totalmap_obstcal_height_.at<uchar>(obstal_points_m) = 2;
    }
//    cv::namedWindow("nine_height_map", CV_NORMAL);
//    cv::imshow("nine_height_map", nine_height_maps_);

    robot_radius_=40;
    // 更新 机器人半径范围
    for (int i = lidar_m.x - robot_radius_/resolution_; i < lidar_m.x + robot_radius_/resolution_; i++)
    {
        for (int j = lidar_m.y - robot_radius_/resolution_; j < lidar_m.y + robot_radius_/resolution_; j++)
        {
            double dist = Dist2d(i - lidar_m.x, j -lidar_m.y)*resolution_;
            if (dist < robot_radius_)
            {
                // 更新free
                double logbel;
                getGridLogBel(j, i, logbel);
                logbel += 2*log_free_;
                setGridLogBel(j, i, logbel);
            }
        }
    }

//    cv::Mat imgErase = cv::Mat::zeros(nine_maps_.rows, nine_maps_.cols, CV_8UC3);

    // 更新 nongroundpoints
    cv::Point2i nonground_points_w;
    cv::Point2i nonground_points_m;
    for (size_t i = 0; i < nonground_points.size(); i++)
    {
        // trans hits points to global world
        nonground_points_w.x = nonground_points[i].x * R[0][0]
                + nonground_points[i].y * R[0][1]
                + nonground_points[i].z * R[0][2]
                + gauss_pos.x;
        nonground_points_w.y = nonground_points[i].x * R[1][0]
                + nonground_points[i].y * R[1][1]
                + nonground_points[i].z * R[1][2]
                + gauss_pos.y;

        worldToNineMap(nonground_points_w, nonground_points_m);


        std::vector<cv::Point2i> line;
        LineBresenham(lidar_m, nonground_points_m, line);
        if (line.size() <=2 )
            continue;

//        double nonground_point_dist = Dist2d(nonground_points_m.y - lidar_m.y, nonground_points_m.x - lidar_m.x)*resolution_;

        for (size_t j = line.size()-2; j > 1; j--)
        {
            double dist = Dist2d(line[j].y - lidar_m.y, line[j].x - lidar_m.x)*resolution_;
            int last_obstal_value = 0;
            if (    last_obstcal_map_.at<uchar>(line[j].y,line[j].x) > 0 ||
                    last_obstcal_map_.at<uchar>(line[j].y,line[j].x-1) > 0 ||
                    last_obstcal_map_.at<uchar>(line[j].y,line[j].x+1) > 0 ||

                    last_obstcal_map_.at<uchar>(line[j].y+1,line[j].x+1) > 0 ||
                    last_obstcal_map_.at<uchar>(line[j].y+1,line[j].x) > 0 ||
                    last_obstcal_map_.at<uchar>(line[j].y+1,line[j].x-1) > 0 ||

                    last_obstcal_map_.at<uchar>(line[j].y-1,line[j].x+1) > 0 ||
                    last_obstcal_map_.at<uchar>(line[j].y-1,line[j].x) > 0 ||
                    last_obstcal_map_.at<uchar>(line[j].y-1,line[j].x-1) > 0)
            {
                last_obstal_value = 255;
            }

            if (matObstacleFlag.at<uchar>(line[j].y,line[j].x) > 0 && last_obstal_value > 0)
            {
                double logbel;
                getGridLogBel(line[j].y, line[j].x, logbel);
                logbel += log_occ_;
                setGridLogBel(line[j].y, line[j].x, logbel);
                continue;
            }

            if(matObstacleFlag.at<uchar>(line[j].y,line[j].x) > 0) continue;


            int height;
            height = std::max(nine_height_maps_.at<uchar>(line[j].y, line[j].x), nine_height_maps_.at<uchar>(line[j].y, line[j].x+1));
            height = std::max(height, (int)nine_height_maps_.at<uchar>(line[j].y, line[j].x-1));
            height = std::max(height, (int)nine_height_maps_.at<uchar>(line[j].y+1, line[j].x));
            height = std::max(height, (int)nine_height_maps_.at<uchar>(line[j].y-1, line[j].x));
            height = std::max(height, (int)nine_height_maps_.at<uchar>(line[j].y-1, line[j].x-1));
            height = std::max(height, (int)nine_height_maps_.at<uchar>(line[j].y-1, line[j].x+1));
            height = std::max(height, (int)nine_height_maps_.at<uchar>(line[j].y+1, line[j].x-1));
            height = std::max(height, (int)nine_height_maps_.at<uchar>(line[j].y+1, line[j].x+1));

//            height = std::max(height)
            //double height_threshold = 120* (ground_point_dist - dist) / ground_point_dist;

            if(nonground_points[i].y<50 && height<100)
                continue;

//            if(height==0)
//            {
//                imgErase.at<cv::Vec3b>(line[j].y, line[j].x)=cv::Vec3b(255,255,255);
//            }
//            else if (height < 20 && height>0)
//                imgErase.at<cv::Vec3b>(line[j].y, line[j].x)=cv::Vec3b(0,255,0);
//            else if (height >=20 && height <= 100)
//                imgErase.at<cv::Vec3b>(line[j].y, line[j].x)=cv::Vec3b(255,0,0);
//            else if (height >100 && height < 160)
//                imgErase.at<cv::Vec3b>(line[j].y, line[j].x)=cv::Vec3b(0,0,255);

            if (height == 0)
            {
                // 更新free
                double logbel;
                getGridLogBel(line[j].y, line[j].x, logbel);
                logbel += log_free_;
                setGridLogBel(line[j].y, line[j].x, logbel);
//                imgErase.at<cv::Vec3b>(line[j].y, line[j].x)=cv::Vec3b(0,255,0);
            }
            else if (height <= 20)  // 台阶部分
            {
//                imgErase.at<cv::Vec3b>(line[j].y, line[j].x)=cv::Vec3b(255,0,0);
                continue;
            }
            else if (height > 100)
            {
                // 更新free
                double logbel;
                getGridLogBel(line[j].y, line[j].x, logbel);
                logbel += 2*log_free_;
                setGridLogBel(line[j].y, line[j].x, logbel);
//                imgErase.at<cv::Vec3b>(line[j].y, line[j].x)=cv::Vec3b(0,255,0);
            }
            else if(height<=100&& height>20 && dist<300)
            {
//                imgErase.at<cv::Vec3b>(line[j].y, line[j].x)=cv::Vec3b(0,0,255);
                continue;
            }
            else if(height<=100&& height>20 && dist<600 && dist>=300)
            {
                // 更新free
                double logbel;
                getGridLogBel(line[j].y, line[j].x, logbel);
                logbel += log_free_;
                setGridLogBel(line[j].y, line[j].x, logbel);

            }
        }
    }

    last_obstcal_map_ = matObstacleFlag.clone();
//    cv::circle(imgErase, lidar_m, 10, cv::Scalar(0,255,255));
//    showMap("nine_map", nine_maps_, lidar_m);
//    cv::namedWindow("erase flag", cv::WINDOW_NORMAL);
//    imshow("erase flag", imgErase);

    return 0;
}

void BuildMap2::printfMat(const cv::Mat &mat)
{
    printf("Mat = \n");
    for (int i = 0; i < mat.rows; i++){
        for (int j = 0; j < mat.cols; j++){
            printf("%f ", mat.at<float>(i, j));
        }
        printf("\n");
    }

}



bool BuildMap2::setGridBel(const int &x, const int &y, const double &bel)
{
    nine_maps_.at<double>(x, y) = bel;
//    totalmap_bel_.at<double>(x, y) = bel;
}

bool BuildMap2::getGridBel(const int &x, const int &y, double &bel)
{
    bel = nine_maps_.at<double>(x, y);
//    bel = totalmap_bel_.at<double>(x, y);
}

bool BuildMap2::setGridLogBel(const int &x, const int &y, const double &log_bel)
{
    double bel = 1.0 - 1.0 / (1 + exp(log_bel));
    if (bel > 0.9) bel = 0.9;
    if (bel < 0.1) bel = 0.1;
    setGridBel(x, y, bel);
}

bool BuildMap2::getGridLogBel(const int &x, const int &y, double &log_bel)
{
    double bel;
    getGridBel(x, y, bel);
    log_bel = log( bel / (1.0-bel) );
}

//void BuildMap::showMap(std::string map_name, cv::Mat img, cv::Point2i robot_pose)
//{
////    cv::Mat map(cv::Size(img.rows, img.cols), CV_8UC1);
//    cv::Mat map = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
//    for(int i = 0; i < img.rows; i++)
//        for (int j = 0; j < img.cols; j++)
//        {
//            double value = img.at<double>(i, j);
//            if (value == 0.5)
//                map.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
//            else if (value < 0.5)
//                map.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
//            else
//                map.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
////            value = value*255;
////            map.at<uchar> (i, j) = 255 - value;
//        }
////    cv::flip(map1, map1, 0);
//    cv::circle(map, robot_pose, 10, cv::Scalar(0));
////    cv::namedWindow(map_name.c_str(), CV_NORMAL);
////    cv::imshow(map_name.c_str(), map);
////    cv::waitKey(1);
//}

void BuildMap2::writeXml(std::string filename, cv::Mat mat)
{
//    printf("channel = %d\n", mat.channels());
//    cv::Mat mat1 = cv::Mat::zeros(cv::Size(littlemap_rows_, littlemap_cols_), CV_64FC1);
//    cv::Mat mat1 = cv::Mat(cv::Size(littlemap_rows_, littlemap_cols_), CV_32FC1, 0.5);
    cv::Mat mat1;
    mat.convertTo(mat1, CV_32FC1);

    cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
//    cv::write(fs, "data", mat);
    fs << "data" << mat1;
    fs.release();
}

void BuildMap2::readXml(std::string filename, cv::Mat &mat)
{
    cv::Mat mat1 = cv::Mat::zeros(cv::Size(littlemap_rows_, littlemap_cols_), CV_32FC1);
    cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
    if (fs.isOpened())
        fs["data"] >> mat1;
    else
        mat1 = cv::Mat(cv::Size(littlemap_rows_, littlemap_cols_), CV_32FC1, 0.5);

    mat1.convertTo(mat, CV_64FC1);
    fs.release();
}

void BuildMap2::writeBin(std::string filename, cv::Mat mat)
{
    FILE *fp;
    fp = fopen(filename.c_str(),"wb");
    if(fp==NULL)
    {
        perror("writeBin file create error!");
        return ;
    }

    for(int i = 0; i < mat.rows; i++)
    {
        for(int j = 0; j < mat.cols; j++)
        {
            double value = mat.at<double>(i, j);
            fwrite(&value, sizeof(double), 1, fp);
        }
    }
    fclose(fp);
}

void BuildMap2::readBin(std::string filename, cv::Mat &mat)
{
    FILE *fp;
    fp = fopen(filename.c_str(),"rb");
    if(fp==NULL)
    {
//        perror("readBin file create error!");
        mat = cv::Mat(cv::Size(littlemap_rows_, littlemap_cols_), CV_64FC1, 0.5);
        return;
    }

    mat = cv::Mat::zeros(cv::Size(littlemap_rows_, littlemap_cols_), CV_64FC1);
    for(int i = 0; i < mat.rows; i++)
    {
        for(int j = 0; j < mat.cols; j++)
        {
            double value;
            fread(&(value), sizeof(double), 1, fp);
            mat.at<double>(i, j) = value;
        }
    }
    fclose(fp);
}

void BuildMap2::savePng(std::string filename, cv::Mat mat)
{
    cv::Mat map1(cv::Size(mat.rows, mat.cols), CV_8UC1);
    for(int i = 0; i < mat.rows; i++)
        for (int j = 0; j < mat.cols; j++)
        {
            int value = mat.at<double>(i, j)*255;
            map1.at<uchar> (i, j) = 255 - value;
        }

    cv::imwrite(filename, map1);
}

void BuildMap2::getRobotMap(cv::Mat &map)
{
    int roi_size = littlemap_rows_;
    int roi_x = robot_pose_m_.x - roi_size/2;
    int roi_y = robot_pose_m_.y - roi_size/2;
    map = cv::Mat::zeros(roi_size, roi_size, CV_8UC1);

    for(int i = 0; i < roi_size; i++)
        for (int j = 0; j < roi_size; j++)
        {
            int value = nine_maps_.at<double>(roi_y + i, roi_x + j)*255;
            map.at<uchar> (i, j) = 255 - value;
        }

}

void BuildMap2::getProbMap(cv::Mat &map)
{
    std::vector<cv::Point2i> littlemap_origins;
    savePreNineMaps();
    getLittleMapOrigins(littlemap_origins);
    combineLittleMaps(littlemap_origins, map);
}


void BuildMap2::getLittleMapOrigins(std::vector<cv::Point2i> &origins)
{
    dirent *ptr;
    DIR *dir;
    dir = opendir(littlemap_save_path_.c_str());
    if (dir == NULL)
    {
        std::cout << "Cannot open dir " << littlemap_save_path_ << std::endl;
        return ;
    }

    std::string str;
    cv::Point2i origin;
    while((ptr=readdir(dir))!=NULL)
    {
        //跳过'.'和'..'两个目录
        if(ptr->d_name[0] == '.')
            continue;

        str = std::string(ptr->d_name);
        if (str.substr(0, 3) == "XS_")
        {
            int idx1 = str.find_first_of("_");
            int idx2 = str.find_last_of("_");
            int idx3 = str.find_first_of(".bin");

            origin.x = std::stoi(str.substr(idx1+1, idx2-idx1-1));
            origin.y = std::stoi(str.substr(idx2+1, idx3-idx2-1));
            origins.push_back(origin);
//            std::cout << "origin [" << origin.x << " " << origin.y << "]" << std::endl;
        }
    }
    closedir(dir);

}

void BuildMap2::combineLittleMaps(std::vector<cv::Point2i> origins,
                                 cv::Mat &map) {
  int minx, miny;
  int maxx, maxy;

  // load range front range.txt
  std::string rang_txt = data_path + "/range.txt";
  std::cout << "rang_txt : " << rang_txt << std::endl;

  std::string line;
  std::ifstream input(rang_txt.c_str());
  std::getline(input, line);
  std::vector<std::string> strv;
  boost::split(strv, line, boost::is_any_of(","));
  std::vector<int> range_value;
  for (size_t i = 0; i < strv.size(); i++) {
    boost::trim(strv[i]);

    std::vector<std::string> valuev;
    boost::split(valuev, strv[i], boost::is_any_of("="));
    boost::trim(valuev[1]);

    range_value.push_back(std::atoi(valuev[1].c_str()));
  }

  minx = range_value[0];
  miny = range_value[1];
  maxx = range_value[2];
  maxy = range_value[3];

  // for (int i = 1; i < origins.size(); i++) {
  //   minx = (origins[i].x < minx) ? origins[i].x : minx;
  //   miny = (origins[i].y < miny) ? origins[i].y : miny;
  //   maxx = (origins[i].x > maxx) ? origins[i].x : maxx;
  //   maxy = (origins[i].y > maxy) ? origins[i].y : maxy;
  // }

  // maxx += littlemap_size_;
  // maxy += littlemap_size_;
  map_range_max_.x = maxx;
  map_range_max_.y = maxy;
  map_range_min_.x = minx;
  map_range_min_.y = miny;
  // LogInfo("littlemap range minx = %d, miny = %d, maxx = %d, maxy = %d\n",
  // minx,
  //         miny, maxx, maxy);
  int cols = (map_range_max_.x - map_range_min_.x) / resolution_;
  int rows = (map_range_max_.y - map_range_min_.y) / resolution_;

  std::cout << "img cols = " << cols << ", rows = " << rows << std::endl;
  map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(128, 128, 128));

  cv::Mat height_map = cv::Mat::zeros(rows, cols, CV_8UC1);

  std::string filename;
  cv::Mat img;
  for (int i = 0; i < origins.size(); i++) {
    int col = (origins[i].x - map_range_min_.x) / resolution_;
    int row = rows -
              (origins[i].y + littlemap_size_ - map_range_min_.y) / resolution_;

    if (col < 0 || col > cols - 1 - img.cols || row < 0 ||
        row > rows - 1 - img.rows) {
      continue;
    }
    //        cout << "col = " << col << ", row = " << row << endl;

    filename = littlemap_save_path_ + "/XS_" + std::to_string(origins[i].x) +
               "_" + std::to_string(origins[i].y) + ".bin";

    //        cout << filename << endl;

    readBin(filename, img);

    cv::Mat tmp = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
    for (int i = 0; i < img.rows; i++)
      for (int j = 0; j < img.cols; j++) {
        double value = img.at<double>(i, j);
        if (value == 0.5)
          tmp.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
        else if (value < 0.5)
          tmp.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
        else
          tmp.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
      }

    cv::Mat roi_img = map(cv::Rect(col, row, img.cols, img.rows));
    tmp.copyTo(roi_img);

    cv::Mat height_tmp;
    std::string height_name = littlemap_save_path_ + "/XS_" +
                              std::to_string(origins[i].x) + "_" +
                              std::to_string(origins[i].y) + ".png";

    height_tmp = cv::imread(height_name.c_str(), -1);
    cv::Mat height_roi_img = height_map(cv::Rect(col, row, img.cols, img.rows));
    height_tmp.copyTo(height_roi_img);
  }

  //    cv::imwrite("height_map.png", height_map);
}


//void BuildMap::combineLittleMaps(std::vector<cv::Point2i> origins, cv::Mat &map)
//{
//    int minx, miny;
//    int maxx, maxy;

//    minx = origins.at(0).x;
//    miny = origins.at(0).y;
//    maxx = origins.at(0).x;
//    maxy = origins.at(0).y;

////    for(int i = 1; i < origins.size(); i++)
////    {
////        minx = (origins[i].x < minx) ? origins[i].x : minx;
////        miny = (origins[i].y < miny) ? origins[i].y : miny;
////        maxx = (origins[i].x > maxx) ? origins[i].x : maxx;
////        maxy = (origins[i].y > maxy) ? origins[i].y : maxy;
////    }

////    maxx += littlemap_size_;
////    maxy += littlemap_size_;

//    map_range_max_.x = maxx;
//    map_range_max_.y = maxy;
//    map_range_min_.x = minx;
//    map_range_min_.y = miny;
////    LogInfo("littlemap range minx = %d, miny = %d, maxx = %d, maxy = %d\n", minx, miny, maxx, maxy);
//    int cols = (map_range_max_.x - map_range_min_.x)/ resolution_;
//    int rows = (map_range_max_.y - map_range_min_.y)/ resolution_;

////    std::cout << "img cols = " << cols << ", rows = " << rows << std::endl;
//    map = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(128, 128, 128));

//    cv::Mat height_map = cv::Mat::zeros(rows, cols, CV_8UC1);

//    std::string filename;
//    for(int i = 0; i < origins.size(); i++)
//    {
//        int col = (origins[i].x - map_range_min_.x)/resolution_;
//        int row = rows - (origins[i].y + littlemap_size_ - map_range_min_.y)/resolution_;

////                cout << "col = " << col << ", row = " << row << endl;

//        filename = littlemap_save_path_ + "/XS_" + std::to_string(origins[i].x) + "_" +
//                std::to_string(origins[i].y) + ".bin";

//                cout << filename << endl;

//        cv::Mat img;
//        readBin(filename, img);

//        cv::Mat tmp = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
//        for(int i = 0; i < img.rows; i++)
//            for (int j = 0; j < img.cols; j++)
//            {
//                double value = img.at<double>(i, j);
//                if (value == 0.5)
//                    tmp.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
//                else if (value < 0.5)
//                    tmp.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
//                else
//                    tmp.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
//            }

//        cv::Mat roi_img = map(cv::Rect(col, row, img.cols, img.rows ));
//        tmp.copyTo(roi_img);

//        cv::Mat height_tmp;
//        std::string height_name = littlemap_save_path_ + "/XS_" + std::to_string(origins[i].x) + "_" +
//                std::to_string(origins[i].y) + ".png";

//        height_tmp = cv::imread(height_name.c_str(), -1);
//        cv::Mat height_roi_img = height_map(cv::Rect(col, row, img.cols, img.rows ));
//        height_tmp.copyTo(height_roi_img);
//    }
////    cv::imwrite("height_map.png", height_map);
//}

void BuildMap2::OnHeightMouse(int event, int x, int y)
{

    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:

    {
        double dist = Dist2d(robot_pose_m_.y - y, robot_pose_m_.x -x)*resolution_;
        printf("Height Map row = %d, col = %d, value = %d robot pose = %d, %d, dist = %f\n", y, x,
               nine_height_maps_.at<uchar>(y, x),
               robot_pose_m_.y, robot_pose_m_.x,
               dist);
    }

        break;

    default:
        break;
    }
}

void BuildMap2::OnHeightMouse(int event, int x, int y, int flags, void *userdata)
{

    BuildMap2* build_map = reinterpret_cast<BuildMap2*>(userdata);
        build_map->OnHeightMouse(event, x, y);


}
