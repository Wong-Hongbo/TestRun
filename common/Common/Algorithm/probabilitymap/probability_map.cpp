#include "probability_map.h"
#include "signal.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "Common.h"

namespace xs {

ProbabilityMap::ProbabilityMap(string data_path)
{
    build_map_ = new BuildMap2(data_path);
    range_max_ = 2000;
    range_min_ = 50;
    grid_size_ = 10;
    scan_num_ = 1800;
    angle_increment_ = static_cast<double>((360.0/scan_num_)/180*M_PI);
    big_slidingwindow_ = new big_slidingwindow_c();

}

ProbabilityMap::~ProbabilityMap()
{
    if (big_slidingwindow_ != NULL)
    {
        delete big_slidingwindow_;
        big_slidingwindow_ = NULL;
    }
}

void ProbabilityMap::GetMap(cv::Mat &map)
{
    build_map_->getRobotMap(map);
}

void ProbabilityMap::UpdateMap(const cv::Point3i gauss_pos,
                               const cv::Vec3d rpy,
                               const std::vector<cv::Point3i> full_points,
                               const std::vector<cv::Point3i> scan_points)
{
//    ShowScan("full_points", full_points);
//    ShowScan("scan", scan_points);
    std::vector<cv::Point3i> scan_points1;
    PointsToScan(scan_points, scan_points1);


    BIG_POSITIVEOBSTACLE_MSG *big_positive_obstacle_msg;
    big_positive_obstacle_msg = new BIG_POSITIVEOBSTACLE_MSG();
//    std::vector<cv::Point3i> full_points_i;
//    for (int i = 0; i < full_points.size(); i++)
//    {
//        cv::Point3i pt;
//        pt.x = static_cast<int>(full_points[i].x*100);
//        pt.y = static_cast<int>(full_points[i].y*100);
//        pt.z = static_cast<int>(full_points[i].z*100);

//        full_points_i.push_back(pt);
//    }
    big_slidingwindow_->compute_obstacle_map_from_points(full_points, big_positive_obstacle_msg, false);

    std::vector<cv::Point3i> obstal_points = big_positive_obstacle_msg->big_PosObPoints;
    build_map_->updateMap(gauss_pos, rpy, scan_points, obstal_points);

    delete big_positive_obstacle_msg;
    big_positive_obstacle_msg = NULL;
}

void ProbabilityMap::UpdateMap(const cv::Point3i gauss_pos,
                               const cv::Vec3d rpy,
                               const std::vector<cv::Point3i> full_points)
{
//    ShowScan("full_points", full_points);
    std::vector<cv::Point3i> ground_cloud;
    std::vector<cv::Point3i> nonground_cloud;
    PointsFilter(full_points, ground_cloud, nonground_cloud);

    BIG_POSITIVEOBSTACLE_MSG *big_positive_obstacle_msg;
    big_positive_obstacle_msg = new BIG_POSITIVEOBSTACLE_MSG();

    big_slidingwindow_->compute_obstacle_map_from_points(full_points, big_positive_obstacle_msg, false);
    std::vector<cv::Point3i> obstal_points = big_positive_obstacle_msg->big_PosObPoints;
    build_map_->updateMap(gauss_pos, rpy, ground_cloud, nonground_cloud, obstal_points);
//    build_map_.updateTotalMap(gauss_pos, rpy, ground_cloud, nonground_cloud, obstal_points);

    delete big_positive_obstacle_msg;
    big_positive_obstacle_msg = NULL;
}

void ProbabilityMap::UpdateMap(const cv::Point3i gauss_pos, const cv::Vec3d rpy, std::string file_name)
{
    std::vector<cv::Point3i> ground_cloud;
    std::vector<cv::Point3i> nonground_cloud;
//    LoadLidarWithPoints(file_name, ground_cloud, nonground_cloud);
    bool rtn = LoadLidarWithScan(file_name, ground_cloud, nonground_cloud);
    if (!rtn)
        return;

    BIG_POSITIVEOBSTACLE_MSG *big_positive_obstacle_msg;
    big_positive_obstacle_msg = new BIG_POSITIVEOBSTACLE_MSG();
    big_slidingwindow_->setInputCloud(inputclouds);
    big_slidingwindow_->compute_obstacle_map(big_positive_obstacle_msg, false);

    std::vector<cv::Point3i> obstal_points = big_positive_obstacle_msg->big_PosObPoints;
    build_map_->updateMap(gauss_pos, rpy, ground_cloud, nonground_cloud, obstal_points);
//    build_map_.updateTotalMap(gauss_pos, rpy, ground_cloud, nonground_cloud, obstal_points);

    delete big_positive_obstacle_msg;
    big_positive_obstacle_msg = NULL;

}

void ProbabilityMap::GetProbMap(cv::Mat &map)
{
    build_map_->getProbMap(map);
}

void ProbabilityMap::LoadLidarWithPoints(const std::string filename, std::vector<cv::Point3i> &all_ground_points,
                               std::vector<cv::Point3i> &all_unground_points)
{
    FILE *fp;
    fp = fopen(filename.c_str(),"rb");
    if(fp==NULL)
    {
        perror("hdladar file open error!");
        return ;
    }
    int ringNum;
    double startTime;   //ms
    double endTime;     //ms
    fread(&(ringNum),sizeof(int),1,fp);
    fread(&(startTime), sizeof(double), 1, fp);
    fread(&(endTime), sizeof(double), 1, fp);

    for(int i=0;i<ringNum;i++)
    {
        for(int j=0;j<PACKETNUM*6;j++)
        {
            fread(&(inputclouds[i][j]),sizeof(PointCoordinate64),1,fp);
        }
    }

    cv::Mat label_mat = cv::Mat(16, 1800, CV_8S, cv::Scalar::all(0));

    float diffX, diffY, diffZ, angle;
//    float hdiffX, hdiffY, hdiffZ, hangle;
    all_ground_points.clear();
    all_unground_points.clear();

    float sum_ground_z = 0;
    int ground_count = 0;

    for (size_t j = 225; j < 1575; ++j){

        for (size_t i = 0; i < 8; ++i){

            if (inputclouds[i][j].x == 0&& inputclouds[i][j].y == 0 && inputclouds[i][j].z == 0)
            {
                continue;
            }

            cv::Point3i pt;
            pt.x = inputclouds[i][j].x;
            pt.y = inputclouds[i][j].y;
            pt.z = inputclouds[i][j].z;

            double range = Dist3d(pt.x, pt.y, pt.z);
            if (range < 20 || range > 2000)
                continue;

            diffX = inputclouds[i][j].x - inputclouds[i+1][j].x;
            diffY = inputclouds[i][j].y - inputclouds[i+1][j].y;
            diffZ = inputclouds[i][j].z - inputclouds[i+1][j].z;
            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;

            if (fabs(angle) < 10 && fabs(diffZ) < 10 && fabs(inputclouds[i][j].z - ground_z) < 10)
            {

                label_mat.at<int8_t>(i,j) = 1;

                all_ground_points.push_back(pt);
                ground_count ++;
                sum_ground_z += pt.z;

//                // i+1
//                pt.x = inputclouds[i+1][j].x;
//                pt.y = inputclouds[i+1][j].y;
//                pt.z = inputclouds[i+1][j].z;

//                if (pt.x == 0 && pt.y == 0 && pt.y == 0)
//                {
//                    continue;
//                }
//                label_mat.at<int8_t>(i+1,j) = 1;

//                all_ground_points.push_back(pt);
//                ground_count ++;
//                sum_ground_z += pt.z;

            }
            else
            {
                // 否则认为是非地面点
                if (label_mat.at<int8_t>(i, j) == 0 && pt.z - ground_z < 40)
                {
                    label_mat.at<int8_t>(i, j) = 10;

                    all_unground_points.push_back(pt);

                }
            }

        }
    }

    ground_z = sum_ground_z / ground_count;
    ShowScan("ground cloud", all_ground_points);
    ShowScan("unground cloud", all_unground_points);
}

bool ProbabilityMap::LoadLidarWithScan(const std::string filename, std::vector<cv::Point3i> &all_ground_points, std::vector<cv::Point3i> &all_unground_points)
{
    FILE *fp;
    fp = fopen(filename.c_str(),"rb");
    if(fp==NULL)
    {
        perror("hdladar file open error!");
        return false;
    }
    int ringNum;
    double startTime;   //ms
    double endTime;     //ms
    fread(&(ringNum),sizeof(int),1,fp);
    fread(&(startTime), sizeof(double), 1, fp);
    fread(&(endTime), sizeof(double), 1, fp);

    for(int i=0;i<ringNum;i++)
    {
        for(int j=0;j<PACKETNUM*6;j++)
        {
            fread(&(inputclouds[i][j]),sizeof(PointCoordinate64),1,fp);
        }
    }

    fclose(fp);
    std::vector<ScanData> scan_datas;
    scan_datas.resize(scan_num_);

    ScanData scan_data[scan_num_];
    memset(scan_data, 0, sizeof(ScanData)*scan_num_);
    for (int i = 0; i < scan_num_; i++)
    {
        scan_data[i].min_range = 2000;
        scan_data[i].max_range = -2000;
        scan_data[i].groundPtExist = false;
        scan_data[i].nongroundPtExist = false;
    }
    std::vector<cv::Point3i> full_points;
    for (int i = 0; i < 16; i++){
        for (size_t j = 225; j < 1575; j++){
            cv::Point3i pt;
            pt.x = inputclouds[i][j].x;
            pt.y = inputclouds[i][j].y;
            pt.z = inputclouds[i][j].z;

            double range = Dist2d(pt.x, pt.y);
            if (range < 20 || range > 2000)
                continue;
            full_points.push_back(pt);

            // 地面点
            if (pt.z > -50 && pt.z < 30)
            {
                if (range > scan_data[j].max_range)
                {
                    scan_data[j].max_range = range;
                    scan_data[j].max_ground_pt = pt; // 存储最远地面点
                    scan_data[j].groundPtExist =true;
                }
            }
            else if (pt.z >= 30 && pt.z <160)
            {
                if (range < scan_data[j].min_range)
                {
                    scan_data[j].min_range = range;
                    scan_data[j].min_nonground_pt = pt; // 存储最近非地面点
                    scan_data[j].nongroundPtExist =true;
                }
            }
        }
    }

    for (int i = 225; i < 1575; i++)
    {
        if(scan_data[i].groundPtExist && scan_data[i].nongroundPtExist==false)
        {
            all_unground_points.push_back(scan_data[i].max_ground_pt);
        }
        else if(scan_data[i].nongroundPtExist && scan_data[i].groundPtExist==false)
        {
            all_unground_points.push_back(scan_data[i].min_nonground_pt);
        }
        else
        {
            if(scan_data[i].max_range<scan_data[i].min_range)
            {
                all_unground_points.push_back(scan_data[i].max_ground_pt);
            }
            else
            {
                all_unground_points.push_back(scan_data[i].min_nonground_pt);
            }
        }
    }
//    ShowScan("full cloud", full_points);
//    ShowScan("ground cloud", all_ground_points);
//    ShowScan("unground cloud", all_unground_points);

}


void ProbabilityMap::PointsToScan(const std::vector<cv::Point3i> &points, std::vector<cv::Point3i> &singlering_points)
{
    double range;
    std::vector<double> ranges;
    ranges.resize(scan_num_);
    ranges.assign(scan_num_, range_max_);

    std::vector<ScanData> scan_datas;
    scan_datas.resize(scan_num_);

    ScanData scan_data[scan_num_];
    memset(scan_data, 0, sizeof(ScanData)*scan_num_);

    for (size_t i = 0; i < points.size(); i++)
    {
        cv::Point3d pt = points[i];

        range = Dist2d(pt.x, pt.y);
        if (range < 20 || range > 2000)
            continue;

        double angle = atan2(pt.y, pt.x);
        int idx = (angle - (-M_PI))/angle_increment_;
        if (idx < 0)
            idx = 1;
        if (idx > scan_num_-1)
            idx = scan_num_-2;

        // 地面点
        if (pt.z > -50 && pt.z < 0)
        {
            if (range > scan_data[idx].max_range)
            {
                scan_data[idx].max_range = range;
                scan_data[idx].max_ground_pt = pt; // 存储最远地面点
            }
        }
        else if (pt.z >= 0 && pt.z < 40)
        {
            if (range < scan_data[idx].min_range)
            {
                scan_data[idx].min_range = range;
                scan_data[idx].min_nonground_pt = pt; // 存储最近非地面点
            }
        }
    }

    for(int i = 0; i < scan_num_; i++)
    {
        // 先判断是否有最近的非地面点， 有则保存
        if (scan_data[i].min_nonground_pt.x != 0 || scan_data[i].min_nonground_pt.y != 0)
        {
            singlering_points.push_back(scan_data[i].min_nonground_pt);
        }
        else if (scan_data[i].min_nonground_pt.x == 0 && scan_data[i].min_nonground_pt.y == 0)
        {
            if (scan_data[i].max_ground_pt.x != 0 || scan_data[i].max_ground_pt.y != 0)
            singlering_points.push_back(scan_data[i].max_ground_pt);
        }
    }

}

void ProbabilityMap::PointsFilter(const std::vector<cv::Point3i> &points,
                                  std::vector<cv::Point3i> &all_ground_points,
                                  std::vector<cv::Point3i> &all_unground_points)
{
    std::vector<ScanData> scan_datas;
    scan_datas.resize(scan_num_);

    ScanData scan_data[scan_num_];
    memset(scan_data, 0, sizeof(ScanData)*scan_num_);
    for (int i = 0; i < scan_num_; i++)
    {
        scan_data[i].min_range = 2000;
        scan_data[i].max_range = -2000;
        scan_data[i].groundPtExist = false;
        scan_data[i].nongroundPtExist = false;
    }

    double angle;
    int idx;
    for (size_t i = 0; i < points.size(); i++)
    {
        angle = atan2(points[i].y-71.4, points[i].x) * 180. / M_PI;
        angle = 360-(angle +90);
        angle = NormalizeAngleToDegree(angle);

        idx = angle/0.2;
        if (idx < 0)
            idx = 0;
        if (idx > scan_num_-1)
            idx = scan_num_-1;

        cv::Point3i pt = points[i];
        double range = Dist2d(pt.x, pt.y);
        if (range < 20 || range > 2000)
            continue;

        // 地面点
        if (pt.z > -50 && pt.z < 30)
        {
            if (range > scan_data[idx].max_range)
            {
                scan_data[idx].max_range = range;
                scan_data[idx].max_ground_pt = pt; // 存储最远地面点
                scan_data[idx].groundPtExist =true;
            }
        }
        else if (pt.z >= 30 && pt.z <180)
        {
            if (range < scan_data[idx].min_range)
            {
                scan_data[idx].min_range = range;
                scan_data[idx].min_nonground_pt = pt; // 存储最近非地面点
                scan_data[idx].nongroundPtExist =true;
            }
        }
    }
    all_unground_points.clear();
    for (int i = 225; i < 1575; i++)
    {
        if(scan_data[i].groundPtExist && scan_data[i].nongroundPtExist==false)
        {
            all_unground_points.push_back(scan_data[i].max_ground_pt);
        }
        else if(scan_data[i].nongroundPtExist && scan_data[i].groundPtExist==false)
        {
            all_unground_points.push_back(scan_data[i].min_nonground_pt);
        }
        else
        {
            if(scan_data[i].max_range<scan_data[i].min_range)
            {
                all_unground_points.push_back(scan_data[i].max_ground_pt);
            }
            else
            {
                all_unground_points.push_back(scan_data[i].min_nonground_pt);
            }
        }
    }

//    ShowScan("unground cloud", all_unground_points);
}

void ProbabilityMap::ShowScan(std::string map_name, const std::vector<cv::Point3i> &points)
{
    cv::Mat scan = cv::Mat::zeros(401, 401, CV_8UC1);
//    cv::Mat line = cv::Mat::zeros(501, 501, CV_8UC1);
    for (int i = 0; i < points.size(); i++)
    {
        if(points[i].x == 0 && points[i].y == 0)
            continue;
        int col = points[i].x / 10 + 200;
        int row = 200 - points[i].y / 10;

        col = Clamp2(col, 0, 400);
        row = Clamp2(row, 0, 400);

        int value = points[i].z;
//        if (value < 0)
//            value = 0;
//        if (value > 255)
//            value = 255;
        value=255;
        scan.at<uchar>(row, col) = value;

    }

    nonground_mat_ = scan.clone();
    cv::imshow(map_name.c_str(), scan);

//    int key1 = cv::waitKey(27);
//    if (key1 == 'p')
//    {
//        cv::setMouseCallback(map_name.c_str(), OnMouse, this);
//        cv::waitKey(-1);
//    }
}

void ProbabilityMap::ShowScan(std::string map_name, const std::vector<cv::Point3d> &points)
{
    cv::Mat scan = cv::Mat::zeros(501, 501, CV_8UC1);
//    cv::Mat line = cv::Mat::zeros(501, 501, CV_8UC1);
    for (int i = 0; i < points.size(); i++)
    {
        if(points[i].x == 0 && points[i].y == 0)
            continue;
        int col = points[i].x / 10 + 250;
        int row = 250 - points[i].y / 10;

        col = Clamp2(col, 0, 500);
        row = Clamp2(row, 0, 500);

        scan.at<uchar>(row, col) = 255;
    }
    cv::imshow(map_name.c_str(), scan);
//    cv::waitKey(1);
}



void ProbabilityMap::OnMouse(int event, int x, int y)
{
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:
    {

        double dist = Dist2d(abs(y - 200), abs(x-200))*10;
        for (int i = y-20; i < y+20; i++)
            for (int j = x-20; j < x+20; j++)
            printf("Nine Map row = %d, col = %d, value = %d, dist = %f\n", i, j, nonground_mat_.at<uchar>(i, j), dist);


        break;
    }

    default:
        break;
    }
}

void ProbabilityMap::OnMouse(int event, int x, int y, int, void *userdata)
{
    ProbabilityMap* build_map = reinterpret_cast<ProbabilityMap*>(userdata);
    build_map->OnMouse(event, x, y);
}



}
