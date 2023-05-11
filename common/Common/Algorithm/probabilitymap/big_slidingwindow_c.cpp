
#include "big_slidingwindow_c.h"
#include <string.h>
#include <opencv2/opencv.hpp>
#include "config/util.h"

BIG_POSITIVEOBSTACLE_MSG::BIG_POSITIVEOBSTACLE_MSG()
{
    int lidar_valid_distance = 2000;
    int resolution = 5;
    int rows = lidar_valid_distance* 2 / resolution  + 1;
    int cols = lidar_valid_distance* 2 / resolution  + 1;
    int size = rows*cols;
    big_PositiveObstacle = new UINT8 [size];
    big_GrayScaleMap = new UINT8 [size];
    big_Tr_GrayScaleMap = new UINT8 [size];


}

BIG_POSITIVEOBSTACLE_MSG::~BIG_POSITIVEOBSTACLE_MSG()
{
    delete [] big_PositiveObstacle;
    delete [] big_GrayScaleMap;
    delete [] big_Tr_GrayScaleMap;
}

big_slidingwindow_c::big_slidingwindow_c()
{
    valid_lidar_distance = 2000;
    resolution = 5;
    img_cols = valid_lidar_distance*2/resolution +1;
    img_rows = valid_lidar_distance*2/resolution +1;
    img_size = img_cols*img_rows;

    min_obstacle_map = new int [img_size];
    max_obstacle_map = new int [img_size];

    min_height_above_150 = new int [img_size];
    max_height_below_150 = new int [img_size];

    tmp_pos_img = new unsigned char [img_size];

    process_32_data = true;
    process_64_data = false;
    scale = resolution/100.0;
}

big_slidingwindow_c::~big_slidingwindow_c()
{
    delete [] min_obstacle_map;
    delete [] max_obstacle_map;
    delete [] min_height_above_150;
    delete [] max_height_below_150;
    delete [] tmp_pos_img;
}



void big_slidingwindow_c::setInputCloud(HDLADARDATA_MSG *HDLadarData)
{
    process_32_data = true;
    process_64_data = false;
    memcpy(inputclouds,&(HDLadarData->HDData[0][0]),sizeof(inputclouds));
}

void big_slidingwindow_c::setInputCloud(MLADARDATA_MSG *MLadarData)
{
    process_32_data = true;
    process_64_data = false;

    int numL=0,numR=0;
    for(int i=0;i<PACKETNUM*6;i++)
        memset(inputclouds[i],0,sizeof(PointCoordinate64)*64);
    if(MLadarData->WorkflagL)
    {
        numL = MLadarData->TotalNumberL<2100?MLadarData->TotalNumberL:2100;
        for(int i=0;i<numL;i++)
        {
            for(int j=0;j<32;j++)
            {
                inputclouds[i][j].x = MLadarData->PointInfoL[i].line[j].x;
                inputclouds[i][j].y = MLadarData->PointInfoL[i].line[j].y;
                inputclouds[i][j].z = MLadarData->PointInfoL[i].line[j].z;
                inputclouds[i][j].Intensity = (unsigned char)((MLadarData->PointInfoL[i].line[j].Intensity));
            }
        }
        for(UINT32 i=numL;i<MLadarData->TotalNumberL;i++)
        {
            for(int j=32;j<64;j++)
            {
                inputclouds[i-2100][j].x = MLadarData->PointInfoL[i].line[j-32].x;
                inputclouds[i-2100][j].y = MLadarData->PointInfoL[i].line[j-32].y;
                inputclouds[i-2100][j].z = MLadarData->PointInfoL[i].line[j-32].z;
//                inputclouds[i-2100][j].angleH = MLadarData->PointInfoL[i].line[j-32].angleH;
//                inputclouds[i-2100][j].angleV = MLadarData->PointInfoL[i].line[j-32].angleV;
//                inputclouds[i-2100][j].realDistance = short(MLadarData->PointInfoL[i].line[j-32].length*100);
                inputclouds[i-2100][j].Intensity = (unsigned char)((MLadarData->PointInfoL[i].line[j-32].Intensity));
            }
        }
    }

//    if(MLadarData->WorkflagR)
//    {
//        numR = MLadarData->TotalNumberR<2100?MLadarData->TotalNumberR:2100;
//        for(int i=0;i<numR;i++)
//        {
//            for(int j=32;j<64;j++)
//            {
//                inputclouds[i][j].x = short(MLadarData->PointInfoR[i].line[j-32].x*100);
//                inputclouds[i][j].y = short(MLadarData->PointInfoR[i].line[j-32].y*100);
//                inputclouds[i][j].z = short(MLadarData->PointInfoR[i].line[j-32].z*100);
//                inputclouds[i][j].angleH = MLadarData->PointInfoR[i].line[j-32].angleH;
//                inputclouds[i][j].angleV = MLadarData->PointInfoR[i].line[j-32].angleV;
//                inputclouds[i][j].realDistance = short(MLadarData->PointInfoR[i].line[j-32].length*100);
//                inputclouds[i][j].Intensity = (unsigned char)((MLadarData->PointInfoR[i].line[j-32].Intensity));
//            }
//        }
//    }
    //    m_InputCloudsNum = cv::max(numL,numR);
}

void big_slidingwindow_c::setInputCloud(PointCoordinate64 points[64][2100])
{
    memcpy(inputclouds, points, sizeof(inputclouds));
}

// 滤除噪点 comment by fux 19-04-04
void big_slidingwindow_c::filter_measurement()
{
    for(int j=0;j<64;j++)
    {
        for (int i=0;i<PACKETNUM*6;i++)
        {
            double l1=0.0;
            double l_before=0.0;
            double l_after=0.0;
            double x,y,z,x_before,y_before,z_before,x_after,y_after,z_after;
            x=double(inputclouds[i][j].x);
            y=double(inputclouds[i][j].y);
            z=double(inputclouds[i][j].z);
            if (i>=1&&i<PACKETNUM*6 - 1)
            {
                l1=sqrt(x*x+y*y+z*z);
                x_before = double(inputclouds[i-1][j].x);
                y_before = double(inputclouds[i-1][j].y);
                z_before = double(inputclouds[i-1][j].z);

                l_before=sqrt(x_before*x_before+y_before*y_before+z_before*z_before);

                x_after = double(inputclouds[i+1][j].x);
                y_after = double(inputclouds[i+1][j].y);
                z_after = double(inputclouds[i+1][j].z);

                l_after  =sqrt(x_after*x_after+y_after*y_after+z_after*z_after);
            }
            else
            {
                int index_bef=( (i-1)+PACKETNUM*6)%(PACKETNUM*6);
                int index_aft=( i+1) % (PACKETNUM*6);
                l1=sqrt(x*x+y*y+z*z);
                x_before = double(inputclouds[index_bef][j].x);
                y_before = double(inputclouds[index_bef][j].y);
                z_before = double(inputclouds[index_bef][j].z);

                l_before=sqrt(x_before*x_before+y_before*y_before+z_before*z_before);

                x_after = double(inputclouds[index_aft][j].x);
                y_after = double(inputclouds[index_aft][j].y);
                z_after = double(inputclouds[index_aft][j].z);

                l_after  =sqrt(x_after*x_after+y_after*y_after+z_after*z_after);
            }
            if (l1>0.001)
            {
                if (fabs(l1-l_before)>100.0 && fabs(l1-l_after)>100.0)
                {
//                    inputclouds[i][j].realDistance=0;
                    inputclouds[i][j].x=0;
                    inputclouds[i][j].y=0;
                    inputclouds[i][j].z=0;

                }
            }
            else
            {
//                inputclouds[i][j].realDistance=0;
                inputclouds[i][j].x=0;
                inputclouds[i][j].y=0;
                inputclouds[i][j].z=0;
            }
        }
    }
}

void big_slidingwindow_c::compute_obstacle_map_from_points(std::vector<cv::Point3i> points, BIG_POSITIVEOBSTACLE_MSG *_PositiveOb_map, bool display)
{
    //    memcpy(inputclouds,&(HDLadarData->HDData[0][0]),sizeof(inputclouds));
    _PositiveOb_map->big_PosObPoints.clear();


    for(int i=0;i<img_size;i++)
    {
        min_obstacle_map[i] = 10000;
        max_obstacle_map[i] = -10000;
        min_height_above_150[i] = 10000;
        max_height_below_150[i] = -10000;
        tmp_pos_img[i] = 0;
    }

    // only need to process 32 ladar lines
    for (size_t i = 0; i < points.size(); i++)
    {
        int x = points[i].x;
        int y = points[i].y;
        int z = points[i].z;

        double angle = atan2(y, x) * 180 / M_PI;
        if (angle > -180 && angle < 0)
            continue;

        if(x*x + y*y<valid_lidar_distance*valid_lidar_distance) // 单位cm，半径50米
        {
            //快递车的盲区范围

            if(x>-50 && x<50 && y<=50 && y>=-50)
                continue;

            //                if(z<-200 || z>300)
            if(z<-100 || z>220) // 去除（-，-1）（2.2，+）
                continue;

            int u = int(x/resolution) + img_cols/2;
            int v = img_rows/2 - int(y/resolution); // 行翻转
            assert(u<img_cols && u>=0 && v<img_rows && v>=0);

            min_obstacle_map[v*img_rows+u] = std::min(min_obstacle_map[v*img_rows+u],int(z));
            max_obstacle_map[v*img_rows+u] = std::max(max_obstacle_map[v*img_rows+u],int(z));
            if(z>150)
                min_height_above_150[v*img_rows+u] = std::min(min_height_above_150[v*img_rows+u],z);
            else
                max_height_below_150[v*img_rows+u] = std::max(max_height_below_150[v*img_rows+u],z);
        }
    }


    for(int y=0;y<img_rows;y++)
    {
        for(int x=0;x<img_cols;x++)
        {
            int tmp = std::max(0,int(max_obstacle_map[y*img_rows+x]-min_obstacle_map[y*img_rows+x]));
            if(tmp>10)
            {
                if(min_height_above_150[y*img_rows+x]==10000)    // 没有高于1.5米的点
                    tmp_pos_img[y*img_rows+x] = 1;
                else if(max_height_below_150[y*img_rows+x]==-10000)  // z > 150只有高于1.5米的点
                    tmp_pos_img[y*img_rows+x] = 2;
                else if(min_height_above_150[y*img_rows+x] - max_height_below_150[y*img_rows+x]<160) //-1到2.2中间有点且大小小于1.6
                    tmp_pos_img[y*img_rows+x] = 1;
                else
                    tmp_pos_img[y*img_rows+x] = 2;   // hanging object
            }
        }
    }


    memset(_PositiveOb_map->big_PositiveObstacle,0,sizeof(unsigned char)*img_size);
    for (int y=2;y<img_rows-2;y++)
    {
        for (int x=2;x<img_cols-2;x++)
        {
            if(tmp_pos_img[y*img_rows+x]==1)
            {
                int num=0;
                // 计算3x3区域为障碍物的数量
                for (int i=y-2;i<=y+2;i++)
                {
                    for (int j=x-2;j<x+2;j++)
                    {
                        if (tmp_pos_img[i*img_rows+j]==1)
                            num++;
                    }
                }

                if(process_64_data)
                {
                    if(num>=2)
                    {
//                        _PositiveOb_map->big_PosObPoints.push_back(cv::Point2i(x,y));
                        _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                    }
                }
                else if(process_32_data)
                {
                    if(num>=1)
                    {
                        _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                        cv::Point3i pw;
                        pw.x = (x - img_cols/2)*resolution;
                        pw.y = (img_rows/2 - y)*resolution;
                        pw.z = max_obstacle_map[y*img_rows + x];
                        double dist = Dist2d(pw.x, pw.y);
                        if (dist > 1200)
                        {
                            if (pw.z > 100)
                            {
                                _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                                _PositiveOb_map->big_PosObPoints.push_back(pw);
                            }
                        }
                        else if (dist > 1000)
                        {
                            if (pw.z > 80)
                            {
                                _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                                _PositiveOb_map->big_PosObPoints.push_back(pw);
                            }
                        }
                        else
                        {
                            _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                            _PositiveOb_map->big_PosObPoints.push_back(pw);
                        }

//                        _PositiveOb_map->big_PosObPoints.push_back(pw);
                        //                        _PositiveOb_map->big_PosObPoints.push_back(cv::Point2i(x,y));
                    }
                }
            }
            //            else if(tmp_pos_img[y*150+x]==2)
            //                _PositiveOb_map->PositiveObstacle[y*150+x]=0;
        }
    }

    if(display)
        displayPosMap(_PositiveOb_map);
}

// 根据inputclouds点云，计算障碍物map，存到_PositiveOb_map->big_PosObPoints
void big_slidingwindow_c::compute_obstacle_map(BIG_POSITIVEOBSTACLE_MSG * _PositiveOb_map, bool display)
{
//    memcpy(inputclouds,&(HDLadarData->HDData[0][0]),sizeof(inputclouds));
    _PositiveOb_map->big_PosObPoints.clear();

    if(process_64_data)
        filter_measurement();

    for(int i=0;i<img_size;i++)
    {
        min_obstacle_map[i] = 10000;
        max_obstacle_map[i] = -10000;
        min_height_above_150[i] = 10000;
        max_height_below_150[i] = -10000;
        tmp_pos_img[i] = 0;
    }

    // only need to process 32 ladar lines
    for (int i=0;i<16;i++)
    {
        for (int j=0;j<PACKETNUM*6;j++)
        {
            int x = int(inputclouds[i][j].x);
            int y = int(inputclouds[i][j].y);
            int z = int(inputclouds[i][j].z);

            double angle = atan2(y, x) * 180 / M_PI;
            if (angle > -180 && angle < 0)
                continue;

            if(x*x + y*y<valid_lidar_distance*valid_lidar_distance) // 单位cm，半径50米
            {
                //快递车的盲区范围

                if(x>-50 && x<50 && y<=50 && y>=-50)
                    continue;

                if(z<-100 || z>220) // 去除（-，-1）（2.2，+）
                    continue;

                int u = int(x/resolution) + img_cols/2;
                int v = img_rows/2 - int(y/resolution); // 行翻转
                assert(u<img_cols && u>=0 && v<img_rows && v>=0);

                min_obstacle_map[v*img_rows+u] = std::min(min_obstacle_map[v*img_rows+u],int(z));
                max_obstacle_map[v*img_rows+u] = std::max(max_obstacle_map[v*img_rows+u],int(z));
                if(z>150)
                    min_height_above_150[v*img_rows+u] = std::min(min_height_above_150[v*img_rows+u],z);
                else
                    max_height_below_150[v*img_rows+u] = std::max(max_height_below_150[v*img_rows+u],z);
            }
        }
    }

    for(int y=0;y<img_rows;y++)
    {
        for(int x=0;x<img_cols;x++)
        {
            int tmp = std::max(0,int(max_obstacle_map[y*img_rows+x]-min_obstacle_map[y*img_rows+x]));
            if(tmp>10)
            {
                if(min_height_above_150[y*img_rows+x]==10000)    // 没有高于1.5米的点
                    tmp_pos_img[y*img_rows+x] = 1;
                else if(max_height_below_150[y*img_rows+x]==-10000)  // z > 150只有高于1.5米的点
                    tmp_pos_img[y*img_rows+x] = 2;
                else if(min_height_above_150[y*img_rows+x] - max_height_below_150[y*img_rows+x]<160) //-1到2.2中间有点且大小小于1.6
                    tmp_pos_img[y*img_rows+x] = 1;
                else
                    tmp_pos_img[y*img_rows+x] = 2;   // hanging object
            }
        }
    }


    memset(_PositiveOb_map->big_PositiveObstacle,0,sizeof(unsigned char)*img_size);
    for (int y=2;y<img_rows-2;y++)
    {
        for (int x=2;x<img_cols-2;x++)
        {
            if(tmp_pos_img[y*img_rows+x]==1)
            {
                int num=0;
                // 计算3x3区域为障碍物的数量
                for (int i=y-2;i<=y+2;i++)
                {
                    for (int j=x-2;j<x+2;j++)
                    {
                        if (tmp_pos_img[i*img_rows+j]==1)
                            num++;
                    }
                }

                if(process_64_data)
                {
                    if(num>=2)
                    {
//                        _PositiveOb_map->big_PosObPoints.push_back(cv::Point2i(x,y));
                        _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                    }
                }
                else if(process_32_data)
                {
                    if(num>=1)
                    {
                        cv::Point3i pw;
                        pw.x = (x - img_cols/2)*resolution;
                        pw.y = (img_rows/2 - y)*resolution;
                        pw.z = max_obstacle_map[y*img_rows + x];
                        double dist = Dist2d(pw.x, pw.y);
                        if (dist > 1200)
                        {
                            if (pw.z > 100)
                            {
                                _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                                _PositiveOb_map->big_PosObPoints.push_back(pw);
                            }
                        }
                        else if (dist > 1000)
                        {
                            if (pw.z > 80)
                            {
                                _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                                _PositiveOb_map->big_PosObPoints.push_back(pw);
                            }
                        }
                        else
                        {
                            _PositiveOb_map->big_PositiveObstacle[y*img_rows+x]=1;
                            _PositiveOb_map->big_PosObPoints.push_back(pw);
                        }
//                        _PositiveOb_map->big_PosObPoints.push_back(cv::Point2i(x,y));
                    }
                }
            }
//            else if(tmp_pos_img[y*150+x]==2)
//                _PositiveOb_map->PositiveObstacle[y*150+x]=0;
        }
    }

    if(display)
        displayPosMap(_PositiveOb_map);
}

void big_slidingwindow_c::compute_grayscale_map(BIG_POSITIVEOBSTACLE_MSG * _GrayScale_map, bool display)
{
    _GrayScale_map->big_GrayScalePoints.clear();
    PointGrayValue pt;
    memset(_GrayScale_map->big_GrayScaleMap,0,sizeof(unsigned char)*img_size);
    double height_min=-100, height_max=220;
    double height_range=height_max-height_min;
    for(int y=0;y<img_rows;y++)
    {
        for(int x=0;x<img_cols;x++)
        {
            float tmp = std::max(0,(max_obstacle_map[y*img_rows+x]-min_obstacle_map[y*img_rows+x]));

            if(tmp>20)
            {
                if((min_height_above_150[y*img_rows+x]==10000)||(max_height_below_150[y*img_rows+x]==-10000)||(min_height_above_150[y*img_rows+x] - max_height_below_150[y*img_rows+x]<160))
                {
                    pt.gray=UINT8((max_obstacle_map[y*img_rows+x]-height_min)/height_range*255);
                    pt.x=(x-img_rows/2)*scale; pt.y=(img_rows/2-y)*scale;
                    _GrayScale_map->big_GrayScalePoints.push_back(pt);
                    _GrayScale_map->big_GrayScaleMap[y*img_rows+x] = pt.gray;
                }
            }
            else if(tmp<=20&&min_obstacle_map[y*img_rows+x]>=height_min&&min_obstacle_map[y*img_rows+x]!=10000)
            {
                pt.x=(x-img_rows/2)*scale; pt.y=(img_rows/2-y)*scale;
                pt.gray=UINT8((max_obstacle_map[y*img_rows+x]-height_min)/height_range*255);
                _GrayScale_map->big_GrayScalePoints.push_back(pt);
                _GrayScale_map->big_GrayScaleMap[y*img_rows+x] = pt.gray;
            }
        }
    }

    if(display)
        displayGrayScaleMap(_GrayScale_map);
}

void big_slidingwindow_c::compute_transformed_grayscale_map(float delta_x,float delta_y,float delta_a,double last_azimuth,BIG_POSITIVEOBSTACLE_MSG * _GrayScale_map)
{
    memset(_GrayScale_map->big_Tr_GrayScaleMap,0,sizeof(unsigned char)*img_size);
    cv::Mat Tr_GrayScaleMap = cv::Mat::zeros(img_rows,img_cols,CV_8UC1);
    for(int y=0;y<img_rows;y++)
    {
        for(int x=0;x<img_cols;x++)
        {
            if(_GrayScale_map->big_GrayScaleMap[y*img_rows+x])
                Tr_GrayScaleMap.at<UINT8>(y,x) = _GrayScale_map->big_GrayScaleMap[y*img_rows+x];
        }
    }

    cv::Mat rotated_img = cv::Mat::zeros(img_rows,img_rows,CV_8UC1);
    cv::Mat M = cv::getRotationMatrix2D(cv::Point2f(img_rows/2,img_rows/2),delta_a/100,1);
    cv::warpAffine(Tr_GrayScaleMap,rotated_img,M,Tr_GrayScaleMap.size(),cv::INTER_NEAREST);

    float need_delta_x = float(sin(last_azimuth/100/180*M_PI)*delta_x - cos(last_azimuth/100/180*M_PI)*delta_y);
    float need_delta_y = float(cos(last_azimuth/100/180*M_PI)*delta_x + sin(last_azimuth/100/180*M_PI)*delta_y);
    int delta_u = int(need_delta_x/resolution);
    int delta_v = int(need_delta_y/resolution);


    for(int y=0;y<img_rows;y++)
    {
        for(int x=0;x<img_cols;x++)
        {
            if(rotated_img.at<UINT8>(y,x)&&delta_u<(img_cols/2+1)&&delta_u>=(-img_cols/2)
                    &&delta_v<=(img_rows/2+1)&&delta_v>(-img_rows/2))
                _GrayScale_map->big_Tr_GrayScaleMap[(y-delta_v)*img_rows+x+delta_u] = rotated_img.at<UINT8>(y,x);
        }
    }

}

void big_slidingwindow_c::displayPosMap(BIG_POSITIVEOBSTACLE_MSG * _PositiveOb_map)
{
    cv::Mat img = cv::Mat::zeros(img_rows,img_cols,CV_8U);
    for(int y=0;y<img_rows;y++)
    {
        for(int x=0;x<img_cols;x++)
        {
            if(_PositiveOb_map->big_PositiveObstacle[y*img_rows+x]==1)
                img.at<unsigned char>(y,x) = 255;
//            else if(tmp_pos_img[y*150+x]==2)
//                img.at<unsigned char>(y,x) = 128;
        }
    }
//    cv::namedWindow("obstacleMap",CV_WINDOW_NORMAL);
    cv::imshow("obstacleMap",img);
//    cv::waitKey(2);
}

void big_slidingwindow_c::displayGrayScaleMap(BIG_POSITIVEOBSTACLE_MSG * _GrayScale_map)
{
    cv::Mat img = cv::Mat::zeros(img_rows,img_cols,CV_8U);
    for(int y=0;y<img_rows;y++)
    {
        for(int x=0;x<img_cols;x++)
        {
            if(_GrayScale_map->big_GrayScaleMap[y*img_rows+x])
                img.at<unsigned char>(y,x) = _GrayScale_map->big_GrayScaleMap[y*img_rows+x];

        }
    }
    cv::namedWindow("GrayScaleMap",CV_WINDOW_NORMAL);
    cv::imshow("GrayScaleMap",img);
//    cv::waitKey(2);
}






