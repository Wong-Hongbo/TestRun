/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     MapTypeGenerate.cc
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:10:24
*/
#include <QDir>

#include "MapTypeGenerate.hh"
#include "Common/Covproject.h"
//#include "Common/Algorithm/map2d/mapper_constuctmap.h"
#include "common/type.h"
#include "map/p_map.h"
#include "common/port.h"

//debug <<--
#include <chrono>
//int g_tmp_for_specil = 0;
//-->>

//栅格地图hit和miss的概率
float hit_value_ = 0.9;
float miss_value_ = 0.1;
std::vector<uint16> hit_table;
std::vector<uint16> miss_table;

MAPTYPEGENERATE::MAPTYPEGENERATE(MAPDATAFRAME* MapDataFrame, SaveConfig *save_config) : mMapDataFrame(MapDataFrame),
    save_config_(save_config)
{
    final_map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    final_2dmap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    MapRange.littlemap_diameter = 8000; //80 meters
    MapRange.bigmap_length = 400000; //1000meter
    MapRange.bigmap_width = 400000; //1000meter
    Ground_filter = new GROUNDFILTER();

    g_seeds_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    g_ground_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    g_not_ground_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    seg_ground_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    seg_not_ground_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_pose_graph.reset(new pcl::PointCloud<Pointlooppose>);
    g_all_pc.reset(new pcl::PointCloud<PointXYZIL>);
    cloud_raw.reset(new pcl::PointCloud<pcl::PointXYZI>);
    //cloud_keyposes_6d_.reset(new pcl::PointCloud<PointXYZIRPYT>());
    cloud_segmap.reset(new pcl::PointCloud<PointXYZRGBIL>());
    //frame_count= 0;

    struct CarModelParams car_params;
    car_params.ground_height =save_config->car_height_;
    struct PolarGridmapParams polar_params;
    myobstaclemap =  new ObstacleMap(car_params,polar_params);

}

MAPTYPEGENERATE::~MAPTYPEGENERATE()
{

}

void MAPTYPEGENERATE::RebuildCloudMap(pcl::PointCloud<pcl::PointXYZI>& cloud_map)
{
    for(size_t id=0; id<mMapDataFrame->cloud_keyframes_v.size(); id++)
    {
        cloud_map += *GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[id].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[id]);
    }
}

bool MAPTYPEGENERATE::GetFolder()
{
//    ReadConfig();
    ReadMapEditConfig();
    std::stringstream filename1;
    filename1 << save_config_->save_path_ << "/2D"  ;
    sub_save_2DPath = filename1.str();
    std::cout << "2d path  : " << sub_save_2DPath << std::endl;
    if(access(filename1.str().c_str(),0)==-1)
    {
        if(mkdir(filename1.str().c_str(),0744)==-1)
        {
            printf("2D folder create error!\n");
            return false;
        }
    }
    std::stringstream filename2;
    filename2 << save_config_->save_path_ << "/pose"  ;
    sub_save_PosePath = filename2.str();
    std::cout << "pose path  : " << sub_save_PosePath << std::endl;

    if(access(filename2.str().c_str(),0)==-1)
    {
        if(mkdir(filename2.str().c_str(),0744)==-1)
        {

            printf("Pose folder create error!\n");
            return false;
        }
    }

    std::stringstream filename3;
    filename3 << save_config_->save_path_ << "/map_tile"  ;
    sub_save_MaptilePath = filename3.str();
    std::cout << "mapdb path  : " << sub_save_MaptilePath << std::endl;
    if(access(filename3.str().c_str(),0)==-1)
    {
        if(mkdir(filename3.str().c_str(),0744)==-1)
        {
            printf("Map tile folder create error!\n");
            return false;
        }
    }


    if(save_config_->save_submap_)
    {
        std::stringstream submap_filename;
        submap_filename << sub_save_MaptilePath << "/submap";
        if(access(submap_filename.str().c_str(),0)==-1)
        {
            if(mkdir(submap_filename.str().c_str(),0744)==-1)
            {
                printf("Map tile folder create error!\n");
                return false;
            }
        }
    }

    std::stringstream filename4;
    filename4 << save_config_->save_path_ << "/keyframe"  ;
    sub_save_keyframePath = filename4.str();
    std::cout << "keyframe path  : " << sub_save_keyframePath << std::endl;
    if(access(filename4.str().c_str(),0)==-1)
    {
        if(mkdir(filename4.str().c_str(),0744)==-1)
        {
            printf("keyframe folder create error!\n");
            return false;
        }
    }

    return true;
}

void MAPTYPEGENERATE::saveProbGridMap(int &progress)
{
    double startTime, endTime;
    //    string path = "/data/MapData/" + uploadSationID + "/keyframe/";
    string path = kf_path +"/";
//    std::cout << "path :" << path << std::endl;
//    std::string pose_pcdfile = path + "keypose.pcd";
//    std::cout << "pose pcd file : " << pose_pcdfile << std::endl;
    //    LOG(INFO)<<"[saveProbGridMap] 获取关键幀数据路径Path=%s\n",pose_pcdfile.c_str());

    pcl::PointCloud<PointType6D>::Ptr keypose(new pcl::PointCloud<PointType6D>);

    *keypose = *mMapDataFrame->cloud_keyposes_6d_ptr;

    int pose_size =keypose->points.size();
    std::cout << "pose size : " << pose_size << std::endl;

    cv::Vec3d rpyd;    // roll pitch yaw
    cv::Point3i gauss_pos;    // x, y, z
    xs::ProbabilityMap* map;
    map = new xs::ProbabilityMap(sub_save_2DPath);
    std::cout << "ProbabilityMap is ok !\n ";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);


    int count=0;
    for(int i = 0; i < pose_size; i++)
    {
        PointType6D point = keypose->points[i];
        gauss_pos.x = static_cast<int>(point.x*100.0f);
        gauss_pos.y = static_cast<int>(point.y*100.0f);
        gauss_pos.z = static_cast<int>(point.z*100.0f);

        rpyd[0] = (point.roll);
        rpyd[1] = (point.pitch);
        rpyd[2] = (point.yaw);

        cloud->clear();

//        std::cout << "index : " << index*i << std::endl;

        std::string cloud_pcd = path + std::to_string(i) + ".pcd";

//        std::cout << "cloud_pcd : " << cloud_pcd << std::endl;

        pcl::io::loadPCDFile(cloud_pcd.c_str(), *cloud);

        //        printf("point size = %d\n", cloud->points.size());

        //
        std::vector<cv::Point3i> full_points;

        for (size_t j = 0; j < cloud->points.size(); j++)
        {
            pcl::PointXYZI cloud_point = cloud->points[j];
            cv::Point3i full_pt;
            full_pt.x = static_cast<int>(cloud_point.x*100);
            full_pt.y = static_cast<int>(cloud_point.y*100);
            full_pt.z = static_cast<int>(cloud_point.z*100);
            full_points.push_back(full_pt);
        }

        //        printf("full_points size = %d\n", full_points.size());
//        startTime=GetCurrentTime();
        map->UpdateMap(gauss_pos, rpyd, full_points);
//        endTime=GetCurrentTime();
        //        printf("time used = %lf\n", endTime - startTime);
        //        std::cout << "grid map size : " << int(74+index*i) << std::endl;
        //        LOG(INFO)<<"grid map index=%d\n",int(74 + index*(i+1)));
        //        task_status.status.progress = int(74 + index*(i+1)) ;

        count++;
        //        cv::waitKey(0);
        //            map.GetMap(curr_img);
        //            cv::imshow("curr_img", curr_img);
        //            cv::waitKey(1);

        /*1 100张，如果占用10％，则是完成10张整体1％：100/10 = 10，100张如果占用5％，则是完成20张整体增加1％：100/5 = 20;
         * 90张，如果占用为9％,则完成10张整体涨1％：90/9 = 10，如果89张，则89/9 = 9张，9张就增加1％，则到81张就增加了9％，会超前完成9％，所以分母＋1，变成完成10张整体上涨1％.
         * 2 这里的progress不是百分比制，而是要总的数量加上10％后的数量，这里只是处理10％中的9％。如果要处理900张，则progress的总数变成900 ＋ 900x0.1 = 990张，10％是99张，这里只处理9％
         * 是990 x 0.09 = 89.1张，即要增加9％，则总数progress要增加89.1张，则每增加1％则需要增加89.1/9 = 9.9张，即每增加9.9张，则总体增加5％，而增加1%则是上面1的算法
         */
        if(0 == i % (pose_size / 9 + 1))//9是因为这个进度条占用为最后的10％，这部分占用为9％，剩下的1％留给for外面，加1是因为整除变小了，如89／9＝9，失去了89 － 9x9 = 8张
        {
            progress += (pose_size * 1.1 * 0.09)/9;
        }
    }

    std::stringstream totalgridmappath;
    totalgridmappath << sub_save_2DPath << "/totalprobmap.png";
    std::cout << "totalgridmappath : " << totalgridmappath.str() << std::endl;

    cv::Mat totalmap;
    map->GetProbMap(totalmap);
    cv::imwrite(totalgridmappath.str(), totalmap);

//    string rmpath = "rm -r " + sub_save_2DPath + "/littlemap";
//    system(rmpath.c_str());
    QDir dir(QString::fromStdString(sub_save_2DPath + "/littlemap"));
    dir.removeRecursively();

    printf("保存占据图结束\n");
}

void MAPTYPEGENERATE::CSF_groundFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
    g_ground_pc->clear();
    g_not_ground_pc->clear();
    seg_ground_pc->clear();
    seg_not_ground_pc->clear();
    g_all_pc->clear();
    myobstaclemap->PointCloudPreprocess(input);
    myobstaclemap->ObstalePointFilter();

    pcl::PointXYZI op;
    for(int i=0; i<myobstaclemap->m_valid_num; ++i)
    {
        op.x = myobstaclemap->m_pointclouds[i].x;
        op.y = myobstaclemap->m_pointclouds[i].y;
        op.z = myobstaclemap->m_pointclouds[i].z;
        op.intensity = myobstaclemap->m_pointclouds[i].intensity;
        if(myobstaclemap->m_pointclouds[i].type == 0)
        {
            //            op.intensity = 0;
            g_ground_pc->push_back(op);
            seg_ground_pc->push_back(op);
            //            show_pc->push_back(op);
        }
        else if(myobstaclemap->m_pointclouds[i].type == 1)
        {
            //            op.intensity = 255;
            //            show_pc->push_back(op);
            g_not_ground_pc->push_back(op);
            seg_not_ground_pc->push_back(op);
        }
        else if(myobstaclemap->m_pointclouds[i].type == 2)
        {
            //            op.intensity = 200;
            //            show_pc->push_back(op);
            g_not_ground_pc->push_back(op);
            seg_not_ground_pc->push_back(op);
        }
        else if(myobstaclemap->m_pointclouds[i].type == 3)
        {
            g_not_ground_pc->push_back(op);
            seg_not_ground_pc->push_back(op);
            //            op.intensity = 50;
            //            show_pc->push_back(op);
        }
    }
}


void MAPTYPEGENERATE::Save2DMap(int &progress, std::vector<std::tuple<int, int>> inner, std::vector<std::tuple<int, int>> outer)
{
    //g_tmp_for_specil = 0;

    if(mMapDataFrame->interactive_mode)
        kf_path = mMapDataFrame->keyframe_path + "/";

    //add by lyx 2022-20-10 <<--
    map_voxel_size = save_config_->map_voxel_size_;
    tile_size = save_config_->tile_size_;
    grid_size = save_config_->grid_size_;
    station_height = save_config_->car_height_;
    ugv_height = save_config_->car_height_;
    save_single_size = save_config_->save_single_size_;
    //-->>

    auto start_time_0 = std::chrono::steady_clock::now();

    //初始化两个table
    hit_table = ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(hit_value_));
    miss_table = ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(miss_value_));

    auto end_time_0 = std::chrono::steady_clock::now();
    auto duration_0 = std::chrono::duration_cast<std::chrono::seconds>(end_time_0 - start_time_0);
    std::cout << "init table time:" << duration_0.count() << " seconds" << std::endl;

    printf("start save map-----------------------------------------\n");
    progress = 0;
    pcl::PointXYZI minPt, maxPt;

    //lyx add 2023-1-31:由原来图片的大小为整个keypose的大小更改为分割部分的大小
    int inner_size = inner.size();
    if(0 < inner_size)
    {
        pcl::PointCloud<PointType3D> cloud_keyposes_3d;
        for(int i = 0; i < mMapDataFrame->cloud_keyposes_3d_ptr->points.size(); i += save_config_->skip_interval_)
        {
            //被删除部分
            bool ignore = false;
            for(int j = 0; j < outer.size(); ++j)
            {
                std::tuple<int, int> outer_tuple = outer[j];
                int start = std::get<0>(outer_tuple);
                int end = std::get<1>(outer_tuple);
                if(i >= start && i <= end)
                {
                    ignore = true;
                    break;
                }
            }

            if(ignore)
            {
                continue;
            }

            //保留部分
            int inner_size = inner.size();
            if(inner_size > 0)
            {
                ignore = true;
                for(int j = 0; j < inner_size; ++j)
                {
                    std::tuple<int, int> tuple = inner[j];
                    int start = std::get<0>(tuple);
                    int end = std::get<1>(tuple);
                    if(i <= end && i >= start)
                    {
                        ignore = false;
                        break;
                    }
                }

                if(ignore)
                {
                    continue;
                }
            }

            cloud_keyposes_3d.push_back(mMapDataFrame->cloud_keyposes_3d_ptr->at(i));

        }//for(int i = 0; i < mMapDataFrame->cloud_keyposes_3d_ptr->points.size(); i += save_config_->skip_interval_)

        auto end_time_0_1 = std::chrono::steady_clock::now();
        auto duration_0_1 = std::chrono::duration_cast<std::chrono::seconds>(end_time_0_1 - end_time_0);
        std::cout << "get sub zone:" << duration_0_1.count() << " seconds" << std::endl;

        pcl::getMinMax3D(cloud_keyposes_3d,minPt,maxPt);
        TileMapInit(cloud_keyposes_3d.makeShared());
    }
    else
    {
        pcl::getMinMax3D(*mMapDataFrame->cloud_keyposes_3d_ptr,minPt,maxPt);
        TileMapInit();
    }

    auto end_time_0_1 = std::chrono::steady_clock::now();
    auto duration_0_1 = std::chrono::duration_cast<std::chrono::seconds>(end_time_0_1 - end_time_0);
    std::cout << "getMinMax3D and TileMapInit:" << duration_0_1.count() << " seconds" << std::endl;

    //std::cout << mMapDataFrame->cloud_keyposes_6d_ptr->size() << std::endl;
    int min_x,min_y,max_x,max_y;
    min_x = int(minPt.x * 100.0 + pose_x);
    min_y = int(minPt.y * 100.0 + pose_y);
    max_x = int(maxPt.x * 100.0 + pose_x);
    max_y = int(maxPt.y * 100.0 + pose_y);

    std::cout << "^^^^^^^^^^^^^^leng:" << max_x - min_x << "cm width:" << max_y - min_y << "cm erea:" << (max_x - min_x)/100 * (max_y - min_y)/100 << std::endl;

    min_x_ = min_x;
    min_y_ = min_y;
    max_x_ = max_x;
    max_y_ = max_y;

//    std::cout << "pose x : " << pose_x << std::endl;
//    std::cout << "pose y : " << pose_y << std::endl;
//    std::cout << "min x  : " << min_x << "min y : " << " " << min_y << std::endl;

    min_x -= MapRange.littlemap_diameter*0.8;  max_x+=MapRange.littlemap_diameter*0.8;
    min_y -=MapRange.littlemap_diameter*0.8;   max_y+=MapRange.littlemap_diameter*0.8;
    int min_x0=min_x%8000;
    if(min_x0<0)  min_x=(min_x/8000-1)*8000;
    else min_x=(min_x/8000)*8000;
    int min_y0=min_y%8000;
    if(min_y0<0)  min_y=(min_y/8000-1)*8000;
    else min_y=(min_y/8000)*8000;

    MapRange.min_x =min_x;    MapRange.min_y =min_y;
    max_x=((max_x-min_x)/1000+1)*1000+min_x;
    max_y=((max_y-min_y)/1000+1)*1000+min_y;
    MapRange.max_x = max_x;  MapRange.max_y = max_y;
    MapRange.totalmap_width = MapRange.max_x-MapRange.min_x;
    MapRange.totalmap_length = MapRange.max_y-MapRange.min_y;
    MapRange.littlemap_number_x = MapRange.bigmap_width/MapRange.littlemap_diameter;
    MapRange.littlemap_number_y = MapRange.bigmap_length/MapRange.littlemap_diameter;

    auto end_time_1 = std::chrono::steady_clock::now();
    auto duration_1 = std::chrono::duration_cast<std::chrono::seconds>(end_time_1 - end_time_0_1);

    std::cout << "computer image size:" << duration_1.count() << " seconds" << std::endl;

    //std::cerr << "save_config_->save_2d_ : " << save_config_->save_2d_ << std::endl;
    //std::cerr << "grid_size "  << grid_size << " MapRange.totalmap_length : " << MapRange.totalmap_length << " " << MapRange.totalmap_width << std::endl;

    //int count_num=0;
    int cache_num=0;
    int all_save_count=0;

    P_map p_map;
    Lidar_range map_range(Eigen::Vector3f::Identity(), Eigen::Vector3f::Zero());
    p_map.p_map_resolution = 0.02;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map_2d(new pcl::PointCloud<pcl::PointXYZI>);

    if(save_config_->save_singlescan_map_)
    {
        save_config_->save_2d_ = 0;

        //<<--
        //寻找有效范围内的点
        pcl::PointCloud<PointType3D> range;
        if(save_config_->inner_.size() > 0)//填写了范围
        {
            for(int i = 0; i < save_config_->inner_.size(); ++i)
            {
                for(int j = std::get<0>(save_config_->inner_[i]);  j <= std::get<1>(save_config_->inner_[i]); ++j)
                {
                    range.push_back(mMapDataFrame->cloud_keyposes_3d_ptr->at(j));
                }
            }

            pcl::PointXYZI minPt, maxPt;
            pcl::getMinMax3D(range,minPt,maxPt);

            int min_x,min_y,max_x,max_y;
            min_x = int(minPt.x * 100.0 + pose_x);
            min_y = int(minPt.y * 100.0 + pose_y);
            max_x = int(maxPt.x * 100.0 + pose_x);
            max_y = int(maxPt.y * 100.0 + pose_y);

            //TileMapInit();

            std::cout << "pose x : " << pose_x << std::endl;
            std::cout << "pose y : " << pose_y << std::endl;
            std::cout << "min x  : " << min_x << "min y : " << " " << min_y << std::endl;

            min_x -= MapRange.littlemap_diameter*0.8;  max_x+=MapRange.littlemap_diameter*0.8;
            min_y -=MapRange.littlemap_diameter*0.8;   max_y+=MapRange.littlemap_diameter*0.8;
            int min_x0=min_x%8000;
            if(min_x0<0)  min_x=(min_x/8000-1)*8000;
            else min_x=(min_x/8000)*8000;
            int min_y0=min_y%8000;
            if(min_y0<0)  min_y=(min_y/8000-1)*8000;
            else min_y=(min_y/8000)*8000;

            MapRange.min_x =min_x;    MapRange.min_y =min_y;
            max_x=((max_x-min_x)/1000+1)*1000+min_x;
            max_y=((max_y-min_y)/1000+1)*1000+min_y;
            MapRange.max_x = max_x;  MapRange.max_y = max_y;
            MapRange.totalmap_width = MapRange.max_x-MapRange.min_x;
            MapRange.totalmap_length = MapRange.max_y-MapRange.min_y;
            MapRange.littlemap_number_x = MapRange.bigmap_width/MapRange.littlemap_diameter;
            MapRange.littlemap_number_y = MapRange.bigmap_length/MapRange.littlemap_diameter;
        }
        //-->>

        Eigen::Vector3f min_point = Eigen::Vector3f((MapRange.min_x-pose_x)/100.0,
                                                    (MapRange.min_y-pose_y)/100.0,
                                                    0.f);

        Eigen::Vector3f max_point = Eigen::Vector3f((MapRange.max_x-pose_x)/100.0,
                                                    (MapRange.max_y-pose_y)/100.0,
                                                    0.f);

        map_range.min_xy = min_point;

        map_range.max_xy = max_point;

        map_range.calculate_vertex();

        p_map.ResetByRange(map_range);

    }//if(save_config_->save_singlescan_map_)
    else
    {
        save_config_->save_2d_ = 1;
    }

    std::cerr << "before if(save_config_->save_2d_)" << std::endl;
    std::cerr << "MapRange.totalmap_length/grid_size:" << MapRange.totalmap_length/grid_size << std::endl;
    std::cerr << "MapRange.totalmap_width/grid_size:" << MapRange.totalmap_width/grid_size << std::endl;

    if(save_config_->save_2d_)
    {
        TotalMap  = cv::Mat::zeros(MapRange.totalmap_length/grid_size, MapRange.totalmap_width/grid_size, CV_8UC1);
    //    m_TotalIntensityMap=cv::Mat::zeros(MapRange.totalmap_length/m_grid_size, MapRange.totalmap_width/m_grid_size, CV_8UC3);
        TotalIntensityMap = TotalMap.clone();
        CountToalMap = cv::Mat::zeros(MapRange.totalmap_length/grid_size, MapRange.totalmap_width/grid_size, CV_8UC1);
        TotalMixMap = cv::Mat::zeros(MapRange.totalmap_length/grid_size, MapRange.totalmap_width/grid_size, CV_8UC1);
    }

    auto end_time_2 = std::chrono::steady_clock::now();
    auto duration_2 = std::chrono::duration_cast<std::chrono::seconds>(end_time_2 - end_time_1);
    std::cout << "create 2d image mat:" << duration_2.count() << " seconds" << std::endl;

    //save_sub_size = save_sub_end_id - save_sub_start_id;
    //算出总的数量
    int all = 0;
    if(inner.size() > 0)
    {
        for(int i = 0; i < inner.size(); ++i)
        {
            int diff = std::get<1>(inner[i]) - std::get<0>(inner[i]) + 1;//加1是因为0到100是有101个数据
            all += diff;
        }
    }
    else
    {
        all = mMapDataFrame->cloud_keyposes_6d_ptr->points.size();
    }

    for(int i = 0; i < outer.size(); ++i)
    {
        int diff = std::get<1>(outer[i]) - std::get<0>(outer[i]) + 1;//加1是因为0到100是有101个数据
        all -= diff;
    }

    int save_sub_size = all;

    //std::cerr << "ground fitler: " << save_config_->use_ground_filter_<< std::endl;
    //std::cout << "save_single_size_: " << save_config_->save_single_size_<< std::endl;
    //std::cout << "use_map_filter_: " << save_config_->use_map_filter_<< std::endl;

    if(save_config_->save_submap_)
    {
        submap_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    }

    for(int i = 0; i < mMapDataFrame->cloud_keyposes_6d_ptr->points.size(); i += save_config_->skip_interval_)
    {
        //被删除部分
        bool ignore = false;
        for(int j = 0; j < outer.size(); ++j)
        {
            std::tuple<int, int> outer_tuple = outer[j];
            int start = std::get<0>(outer_tuple);
            int end = std::get<1>(outer_tuple);
            if(i >= start && i <= end)
            {
                ignore = true;
                break;
            }
        }

        if(ignore)
        {
            continue;
        }

        //保留部分
        int inner_size = inner.size();
        if(inner_size > 0)
        {
            ignore = true;
            for(int j = 0; j < inner_size; ++j)
            {
                std::tuple<int, int> tuple = inner[j];
                int start = std::get<0>(tuple);
                int end = std::get<1>(tuple);
                if(i <= end && i >= start)
                {
                    ignore = false;
                    break;
                }
            }

            if(ignore)
            {
                continue;
            }
        }

        progress = i;

//        if(/*use_cut_save_map*/save_config_->user_cut_save_type_)
//        {
//            if(i < save_sub_start_id || i > save_sub_end_id)
//            {
//                continue;
//            }
//        }

        //save_sub_size = save_sub_end_id - save_sub_start_id;

        //temp for special <<-- //
//        if(0 < i && i < 19942)
//        {
//            ugv_height = 1.8;
//            //std::cerr << i << " ============ ugv_height: " << ugv_height << std::endl;
//        }
//        else/* if(3077 - 10 < i < 3077 +10)*/
//        {
//            ugv_height = 0;
//        }
        //-->>

        std::string cloud_path = kf_path + std::to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZI>);

        //debug <<--
        if(0 != pcl::io::loadPCDFile(cloud_path.c_str(), *cloud_raw))
        {
            std::cerr << "load " << cloud_path << " failed" << std::endl;
        }

        //std::cout << "load " << cloud_path << " success" << std::endl;
        //==>>
        //pcl::io::loadPCDFile(cloud_path.c_str(), *cloud_raw);
        //-->>

        //lyx add 2023-2-9<<
        if(save_config_->save_submap_)
        {
            bool is_inside = false;
            std::vector<std::tuple<int, int>> submap_inner = save_config_->submap_inner_;
            for(int j = 0; j < submap_inner.size(); ++j)
            {
                std::tuple<int, int> tuple = submap_inner[j];
                int start = std::get<0>(tuple);
                int end = std::get<1>(tuple);
                if(i <= end && i >= start)
                {
                    is_inside = true;
                    break;
                }
            }

            if(is_inside)
            {
                *submap_ += *GetTransformPoint3DCloud(cloud_raw, mMapDataFrame->cloud_keyposes_6d_ptr->points[i]);
            }

        }
        //-->>

        Eigen::AngleAxisf rotation_x(mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rotation_y(mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rotation_z(mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).yaw, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f translation(mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).x, mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).y, mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).z);
        Eigen::Matrix4f T = (translation * rotation_z * rotation_y * rotation_x).matrix();

        std::array<float, 6> pose_3d = {mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).x, mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).y, mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).z,
                                        mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).yaw, mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).pitch, mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).roll};
        Eigen::Matrix4f kf_pose = array_to_matrix_3d(pose_3d);

        //2022-9-22 add <<--
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_trans(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_trans(new pcl::PointCloud<pcl::PointXYZI>);

        if (save_config_->save_segmap_)
        {
            CSF_groundFilter(cloud_raw);

            PointXYZRGBIL pt;

            *ground_trans += *seg_ground_pc;
//            pcl::io::savePCDFileBinaryCompressed("ground_trans.pcd",*ground_trans);

            cloud_segmap->clear();
            for(int i=0;i<ground_trans->size();i++)
            {
                pt.label=1;
                pt.x = ground_trans->points[i].x;
                pt.y = ground_trans->points[i].y;
                pt.z = ground_trans->points[i].z;
                pt.intensity = ground_trans->points[i].intensity;
                pt.r = 0;
                pt.g = 0;
                pt.b = 0;

                cloud_segmap->push_back(pt);
            }

            *no_ground_trans += *seg_not_ground_pc;

            for(int i=0;i<no_ground_trans->size();i++)
            {
                pt.label=0;
                pt.x = no_ground_trans->points[i].x;
                pt.y = no_ground_trans->points[i].y;
                pt.z = no_ground_trans->points[i].z;
                pt.intensity = no_ground_trans->points[i].intensity;
                pt.r = 0;
                pt.g = 0;
                pt.b = 0;
                cloud_segmap->push_back(pt);
            }

            //string scanPath_seg = seg_kf_path + "/" +  std::to_string(i) + ".pcd";
            //pcl::io::savePCDFileBinaryCompressed(scanPath_seg,*cloud_segmap);

            if(save_config_->save_2d_)
                Update2DMap(GetTransformPoint2DCloud(*ground_trans, T),GetTransformPoint2DCloud(*no_ground_trans, T));

            //2022-11-16 for special <<--
//            if(save_config_->save_2d_)
//            {
//                if(!ground_trans->empty())
//                {
//                    float sum = 0.0;
//                    for(int j = 0; j < ground_trans->size(); ++j)
//                    {
//                        sum += ground_trans->at(j).z;
//                    }

//                    float average = sum / ground_trans->size();
//                    std::cerr << "i: " << i << " average: " << average << std::endl;

//                    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud_trans(new pcl::PointCloud<pcl::PointXYZI>);
//                    if(average < -1.5)
//                    {
//                        RPYpose pose;
//                        pose.x = 0;
//                        pose.y = 0;
//                        pose.z = 1.8;
//                        pose.roll = 0;
//                        pose.pitch = 0;
//                        pose.yaw =  0;
//                        Eigen::Matrix4f matrix_pose;
//                        RPYposeToMatrix(pose,matrix_pose);

//                        pcl::transformPointCloud(*cloud_raw,*raw_cloud_trans,matrix_pose);

//                        RPYpose height_pose;
//                        pose.x = 0;
//                        pose.y = 0;
//                        pose.z = -1.8;
//                        pose.roll = 0;
//                        pose.pitch = 0;
//                        pose.yaw =  0;
//                        RPYposeToMatrix(height_pose,matrix_pose);
//                        Eigen::Matrix4f final_matrix = matrix_pose * T;
//                        Update2DMap(GetTransformPoint2DCloud(*raw_cloud_trans, final_matrix));
//                    }
//                    else
//                    {
//                        Update2DMap(GetTransformPoint2DCloud(*cloud_raw, T));
//                    }

//                }
//                else
//                {
//                    Update2DMap(GetTransformPoint2DCloud(*cloud_raw, T));
//                }
//            }//if(save_config_->save_2d_)
        }//if (save_config_->save_segmap_)

        if(save_config_->save_2d_)
        {
            //std::cerr << "++++before Update2DMap" << std::endl;
            Update2DMap(GetTransformPoint2DCloud(*cloud_raw, T));
            //std::cerr << "++++end Update2DMap" << std::endl;
        }



        if(save_config_->save_singlescan_map_)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_raw_trans(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*cloud_raw,*cloud_raw_trans,kf_pose);
            Point_cloud2d inserted_scan = PclToPointcloud2d(*cloud_raw_trans, kf_pose);
            p_map.addscan(inserted_scan, hit_table, miss_table);
            *cloud_map_2d += *cloud_raw_trans;
        }

        //std::cerr << "save_config_->save_map_tile_:" << save_config_->save_map_tile_ << std::endl;

        if(save_config_->save_map_tile_)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>);

            if(/*use_ground_filter*/save_config_->use_ground_filter_)
            {
                //2022-9-22 <<--
                //auto start_time_0 = std::chrono::steady_clock::now();
                Ground_filter->GroundFilterRun(*cloud_raw);
                downSizeAllMap.setLeafSize(map_voxel_size*2,map_voxel_size*2,map_voxel_size*2);
                downSizeAllMap.setInputCloud(Ground_filter->g_ground_pc);
                downSizeAllMap.filter(*Ground_filter->g_ground_pc);

                downSizeAllMap.setLeafSize(map_voxel_size,map_voxel_size,map_voxel_size);
                downSizeAllMap.setInputCloud(Ground_filter->g_not_ground_pc);
                downSizeAllMap.filter(*Ground_filter->g_not_ground_pc);
                *cloud_filter += *Ground_filter->g_not_ground_pc;
                *cloud_filter += *Ground_filter->g_ground_pc;

//                auto end_time_0 = std::chrono::steady_clock::now();
//                auto duration_0 = std::chrono::duration_cast<std::chrono::microseconds>(end_time_0 - start_time_0);
//                std::cout << "old:" << duration_0.count() << " microseconds" << std::endl;
                //=======>>>>>>>>
                //auto start_time = std::chrono::steady_clock::now();
//                CSF_groundFilter(cloud_raw);
//                downSizeAllMap.setLeafSize(map_voxel_size*2,map_voxel_size*2,map_voxel_size*2);
//                downSizeAllMap.setInputCloud(g_ground_pc);
//                downSizeAllMap.filter(*g_ground_pc);

//                downSizeAllMap.setLeafSize(map_voxel_size,map_voxel_size,map_voxel_size);
//                downSizeAllMap.setInputCloud(g_not_ground_pc);
//                downSizeAllMap.filter(*g_not_ground_pc);
//                *cloud_filter += *g_not_ground_pc;
//                *cloud_filter += *g_ground_pc;

//                auto end_time = std::chrono::steady_clock::now();
//                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//                std::cout << "new:" << duration.count() << " microseconds" << std::endl;
                //-->>
                *cloud_map += *GetTransformPoint3DCloud(cloud_filter, mMapDataFrame->cloud_keyposes_6d_ptr->points[i]);
            }
            else
            {
                if(/*use_map_filter*/save_config_->use_map_filter_)
                {
                    downSizeAllMap.setLeafSize(map_voxel_size,map_voxel_size,map_voxel_size);
                    downSizeAllMap.setInputCloud(cloud_raw);
                    downSizeAllMap.filter(*cloud_raw);
                    *cloud_map += *GetTransformPoint3DCloud(cloud_raw, mMapDataFrame->cloud_keyposes_6d_ptr->points[i]);
                }
                else
                    *cloud_map += *GetTransformPoint3DCloud(cloud_raw, mMapDataFrame->cloud_keyposes_6d_ptr->points[i]);
            }

            if(cache_num < save_single_size/*save_config_->save_single_size_*/)
            {
                for (pcl::PointCloud<pcl::PointXYZI>::const_iterator p = cloud_map->begin(); p != cloud_map->end(); p++)
                {
                    int idx = static_cast<int>(floor((p->x - static_cast<float>(ind_x_min*tile_size)) / tile_size));
                    int idy = static_cast<int>(floor((p->y - static_cast<float>(ind_y_min*tile_size)) / tile_size));

                    int id = idy * delta_allInd_size_x + idx;

                    if( id>0 && (id <grids.size()))
                    {
                        const pcl::PointXYZI &tmp = *p;
                        grids.at(id).cloud.points.push_back(tmp);
                    }
                }
            }
            else
            {
                std::cerr << "======SaveTileMapCache 2" << std::endl;
                SaveTileMapCache();
                cache_num=0;
                all_save_count++;
            }

            cache_num++;
            //count_num++;

        }//if(save_config_->save_map_tile_)
    }//for

    if(all_save_count==0 )
    {
        std::cerr << "======SaveTileMapCache 1" << std::endl;
        SaveTileMapCache();
    }
    else if(all_save_count>0 )
    {
        int diff_num;
        if(/*use_cut_save_map*/save_config_->user_cut_save_type_)
            diff_num  = save_sub_size - all_save_count * save_config_->save_single_size_;
        else
            diff_num = mMapDataFrame->cloud_keyposes_6d_ptr->points.size()-all_save_count * save_config_->save_single_size_;

        std::cout << "diff_num : " << diff_num << std::endl;
        if(diff_num < save_config_->save_single_size_)
        {
            //std::cout << "3 " << std::endl;
            std::cerr << "======SaveTileMapCache 3" << std::endl;
            SaveTileMapCache();
        }
    }

    if(save_config_->save_2d_)
    {
        std::cout << "dfdf-1" << std::endl;
        std::stringstream rangepath;
        rangepath << sub_save_2DPath << "/range.txt";

        FILE *fp=fopen(rangepath.str().data(),"w");

        fprintf(fp, "minx = %d , miny = %d, maxx = %d, maxy = %d, grid = %d ,basex = %d,basey = %d ,zone = %d\n", (int)MapRange.min_x, (int)MapRange.min_y,
                (int)MapRange.max_x, (int)MapRange.max_y,(int)grid_size,int(pose_x),int(pose_y),int(ugv_zone));
        fclose(fp);

        std::stringstream totalmappath;
        totalmappath << sub_save_2DPath << "/totalmap.png";

        std::stringstream totalmappath_seg;
        totalmappath_seg << sub_save_2DPath << "/totalmap_seg.png";

        std::stringstream intenistymappath;
        intenistymappath << sub_save_2DPath << "/intmap.png";

        std::cout << "intensity map path : " << intenistymappath.str() << std::endl;

//        //debug <<--
//        std::cout << "sizeof(CV_8UC1): " << sizeof(CV_8UC1) << std::endl;
//        int long long rowmcol = TotalMap.rows * TotalMap.cols;
//        std::cout << "TotalMap.rows * TotalMap.cols " << rowmcol << std::endl;
//        long long size_d = TotalMap.rows * TotalMap.cols * sizeof(CV_8UC1);
//        std::cout << "totalmap size:" << size_d << " " << rowmcol/(1024 * 1024) << "M" << std::endl;
//        //-->>

        cv::imwrite(totalmappath.str(),TotalMap);

        //debug <<--
        std::cout << "end write totomap.png" << std::endl;
        //-->>
        cv::imwrite(totalmappath_seg.str(),TotalMixMap);

        cv::imwrite(intenistymappath.str(),TotalIntensityMap);
        //std::cout << "dfdf-2" << std::endl;
    }

    // 是否输出2d单线地图-室内
    if(save_config_->save_singlescan_map_)
    {
        // 保存2d定位地图png格式
        std::string map_path = sub_save_2DPath + "/map.png";
        MAPPER_INFO("wirting map data to " << map_path);
        MAPPER_INFO("map resolution : " << p_map.p_map_resolution);
        MAPPER_INFO("map size : " << p_map.p_map_width << " * " << p_map.p_map_height);
        cv::Mat img(p_map.p_map_height, p_map.p_map_width, CV_8UC1, cv::Scalar(205));
        cv::Mat pimg(p_map.p_map_height, p_map.p_map_width, CV_8UC1, cv::Scalar(0));
        for(unsigned int y = 0; y < p_map.p_map_height; y++) {
            for(unsigned int x = 0; x < p_map.p_map_width; x++) {
                unsigned int i = x + (p_map.p_map_height - y - 1) * p_map.p_map_width;
                   auto value = p_map.p_map_cells[i];
                   if(value.visit == -1){
                       img.at<uchar>(y, x) = 205;
                       pimg.at<uchar>(y, x) = 0;
                       continue;
                   }
                   float cell_p = (float)value.hit_num/(float)value.visit;
                   pimg.at<uchar>(y, x) = cell_p * 254 + 1;
                   if(cell_p > 0.25){
                       img.at<uchar>(y, x) = 000;
                   }
                   else{
                       img.at<uchar>(y, x) = 254;
                   }
            }
        }
        std::stringstream totalmappath;
        totalmappath << sub_save_2DPath << "/totalmap.png";
        cv::imwrite(totalmappath.str(), img);

        // 保存2d点云地图，计算得分用
        if(cloud_map_2d->size()>0){
            pcl::io::savePCDFile(sub_save_2DPath + "/pclmap.pcd", *cloud_map_2d);
        }

        // 输出range.txt
        std::stringstream rangepath;
        rangepath << sub_save_2DPath << "/range.txt";
        FILE *fp=fopen(rangepath.str().data(),"w");
        int ocupy_map = 1;
        fprintf(fp, "minx = %d , miny = %d, maxx = %d, maxy = %d, grid = %d ,basex = %d,basey = %d ,zone = %d ,ocupy_map = %d\n", (int)MapRange.min_x, (int)MapRange.min_y,
                (int)MapRange.max_x, (int)MapRange.max_y,(int)(round(p_map.p_map_resolution*100.0)),int(pose_x),int(pose_y),int(ugv_zone),int(ocupy_map));

        fclose(fp);
    }

    if(mMapDataFrame->cloud_loop_index->size()>0)
        pcl::io::savePCDFileBinaryCompressed(sub_save_keyframePath + "/looppose.pcd",*mMapDataFrame->cloud_loop_index);
    pcl::io::savePCDFileBinaryCompressed(sub_save_keyframePath + "/keypose.pcd",*mMapDataFrame->cloud_keyposes_6d_ptr);

    if(mMapDataFrame->cloud_pose_graph_ptr->size()>0)
        pcl::io::savePCDFile(sub_save_keyframePath + "/inner_info.pcd",*mMapDataFrame->cloud_pose_graph_ptr);

    printf("save keypose inner_info pcd end\n");

    std::stringstream ConfigPath;
    ConfigPath << sub_save_PosePath << "/pose_mapper.txt";

    FILE *fp2=fopen(ConfigPath.str().data(),"w");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int i=0;i<mMapDataFrame->cloud_keyposes_6d_ptr->size();i++)
    {
        int azimuth = int((mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).yaw+M_PI/2)*(18000/M_PI))%36000;

        if(azimuth<0)
        {
            azimuth+=36000;
        }

        double gauss_x ,gauss_y,height;
        double lon,lat;
        pcl::PointXYZ pt;
        gauss_x  = mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).x*100.0f +pose_x;
        gauss_y  = mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).y*100.0f +pose_y;
        height = mMapDataFrame->cloud_keyposes_6d_ptr->points.at(i).z*100.0f;
        double guass_x_addbase = gauss_x/100.0f +BASE_X;
        double guass_y_addbase = gauss_y/100.0f +BASE_Y;
        GaussProjInvCal(guass_x_addbase,guass_y_addbase,&lon,&lat,ugv_zone);
        //        std::cout << "lon : " << lon << std::endl;
        //        std::cout << "lat : " << lat << std::endl;
        pt.x = lon;
        pt.y = lat;
        pt.z = height/100.0f;
        cloud.push_back(pt);
        //        pcl::io::savePCDFileBinaryCompressed(saveKeyFrame+ "/" + std::to_string(i) + ".pcd",cloud_keyframes_[i]);  //保存当前关键幀结果
        fprintf(fp2, "%lf %lf %ld %lf \n" ,gauss_x,gauss_y,azimuth,height);
    }

    fclose(fp2);
    WriteConfig();
}

void MAPTYPEGENERATE::SaveTileMapCache()
{
//    int points_num = 0;
//    double index = 15.0f/grid_num;
    auto t1 = std::chrono::system_clock::now();

    for(int i=0;i<grid_num;i++)
    {
        if(grids.at(i).bExistFile && grids.at(i).cloud.size()>0)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr disk_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::io::loadPCDFile(grids.at(i).filename,*disk_cloud);
            grids.at(i).cloud += *disk_cloud;
            std::cout << "Wrote disk again " << grids.at(i).cloud.points.size() << " points to "
                      << grids.at(i).filename << "."  << std::endl;
            pcl::io::savePCDFileBinaryCompressed(grids.at(i).filename, grids.at(i).cloud);
            grids.at(i).cloud.clear();
        }
        else
        {
            if(grids.at(i).cloud.size()>10)
            {
                //            LOG(INFO)<<"map tile progress :%d =====================  " , int(45+ index*i));
//                task_status.status.progress=int(45 +index*i);
//                memcpy(MapperResponse_data->MsgContent,&task_status,sizeof(MapperStatus));
//                MAPPERRESPONSE_CHANNEL->write(MapperResponse_data);
                pcl::io::savePCDFileBinaryCompressed(grids.at(i).filename, grids.at(i).cloud);
                std::cout << "new Wrote  " << grids.at(i).cloud.points.size() << " points to "
                          << grids.at(i).filename << "."  << std::endl ;
                grids.at(i).bExistFile = true;
                grids.at(i).cloud.clear();
                //            usleep(200000);
            }
        }
    }

    auto t2 = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;
    std::cout  << "using time : " << diff << "ms"  << std::endl;
}

void MAPTYPEGENERATE::TileMapInit()
{
    pcl::PointXYZI minPt,maxPt;
    pcl::getMinMax3D(*mMapDataFrame->cloud_keyposes_3d_ptr,minPt,maxPt);

    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    double min_x,min_y,max_x,max_y;
    min_x = minPt.x-50;
    min_y = minPt.y-50;
    max_x = maxPt.x+50;
    max_y = maxPt.y+50;

    int deltaX_min = min_x;
    int deltaY_min = min_y;
    ind_x_min = int(floor(deltaX_min/tile_size));
    ind_y_min = int(floor(deltaY_min/tile_size));

    int deltaX_max =  max_x;
    int deltaY_max =  max_y;
    ind_x_max = int(ceil(deltaX_max/tile_size));
    ind_y_max = int(ceil(deltaY_max/tile_size));

    delta_allInd_size_x = ind_x_max - ind_x_min/* + 1*/;
    delta_allInd_size_y = ind_y_max - ind_y_min/* + 1*/;

    grid_num = delta_allInd_size_y *  delta_allInd_size_x;

    grids.resize(grid_num);

    for(int j=0; j< delta_allInd_size_y ; j++)
    {
        for(int i=0; i< delta_allInd_size_x ; i++)
        {
            int id = delta_allInd_size_x * j + i;
            grids.at(id).idx = ind_x_min + i ;
            grids.at(id).idy = ind_y_min + j ;
            grids.at(id).xlx = ind_x_min * tile_size + i * tile_size;   // 方格的四个定点
            grids.at(id).ylx = ind_y_min * tile_size + j * tile_size;
            grids.at(id).xrs = ind_x_min * tile_size + (i+1) * tile_size;
            grids.at(id).yrs = ind_y_min * tile_size + (j+1) * tile_size;
            grids.at(id).filename = sub_save_MaptilePath +"/" + "Tile_" + std::to_string(grids.at(id).idx) + "_" + std::to_string(grids.at(id).idy) + ".pcd";
            //std::cout << "grid file name : "  << grids.at(id).filename << std::endl;
        }
    }
}

void MAPTYPEGENERATE::TileMapInit(pcl::PointCloud<PointType3D>::Ptr cloud_keyposes_3d_ptr)
{
    pcl::PointXYZI minPt,maxPt;
    pcl::getMinMax3D(*cloud_keyposes_3d_ptr,minPt,maxPt);

    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    double min_x,min_y,max_x,max_y;
    min_x = minPt.x-50;
    min_y = minPt.y-50;
    max_x = maxPt.x+50;
    max_y = maxPt.y+50;

    int deltaX_min = min_x;
    int deltaY_min = min_y;
    ind_x_min = int(floor(deltaX_min/tile_size));
    ind_y_min = int(floor(deltaY_min/tile_size));

    int deltaX_max =  max_x;
    int deltaY_max =  max_y;
    ind_x_max = int(ceil(deltaX_max/tile_size));
    ind_y_max = int(ceil(deltaY_max/tile_size));

    delta_allInd_size_x = ind_x_max - ind_x_min/* + 1*/;
    delta_allInd_size_y = ind_y_max - ind_y_min/* + 1*/;

    grid_num = delta_allInd_size_y *  delta_allInd_size_x;

    grids.resize(grid_num);

    for(int j=0; j< delta_allInd_size_y ; j++)
    {
        for(int i=0; i< delta_allInd_size_x ; i++)
        {
            int id = delta_allInd_size_x * j + i;
            grids.at(id).idx = ind_x_min + i ;
            grids.at(id).idy = ind_y_min + j ;
            grids.at(id).xlx = ind_x_min * tile_size + i * tile_size;   // 方格的四个定点
            grids.at(id).ylx = ind_y_min * tile_size + j * tile_size;
            grids.at(id).xrs = ind_x_min * tile_size + (i+1) * tile_size;
            grids.at(id).yrs = ind_y_min * tile_size + (j+1) * tile_size;
            grids.at(id).filename = sub_save_MaptilePath +"/" + "Tile_" + std::to_string(grids.at(id).idx) + "_" + std::to_string(grids.at(id).idy) + ".pcd";
            //std::cout << "grid file name : "  << grids.at(id).filename << std::endl;
        }
    }
}

void MAPTYPEGENERATE::Update2DMap(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn)
{
//    pcl::PointXYZI p;
    //std::cerr << "----in Update2DMap" << std::endl;

    int intensity;
    double x,y,z;
    double px,py,pz;
    for (int i = 0; i < cloudIn->points.size(); ++i)
    {
        x = cloudIn->points[i].x;
        y = cloudIn->points[i].y;
        z = cloudIn->points[i].z;

        px = x+pose_x;
        py = y+pose_y;
        pz = z;

        intensity = cloudIn->points[i].intensity;

        if(intensity>30)
            intensity=30;
        if(intensity<10)
            intensity=10;
        intensity = intensity/30.0f*255;

        int total_x = (px-MapRange.min_x)/grid_size;
        int total_y = TotalMap.rows-1-(py-MapRange.min_y)/grid_size;
        int grey = int((pz+1.0)*255/(2.2+1.0));

        if(total_x>=0&&total_x<TotalMap.cols&&total_y>=0&&total_y<TotalMap.rows)
        {
            //if(grey > m_TotalMap.at<uchar>(total_y, total_x)) {
            if(1)
            {
                int avg_val = TotalMap.at<uchar>(total_y, total_x);
                int count_val = CountToalMap.at<uchar>(total_y, total_x);
                if(count_val >= 20) {
                    avg_val = (avg_val*(count_val - 1) + grey) / count_val;
                }
                else {
                    int sum_val = avg_val * count_val + grey;
                    count_val ++;
                    avg_val = sum_val / count_val;
                    CountToalMap.at<uchar>(total_y, total_x) = count_val;
                }
                if(intensity>TotalIntensityMap.at<uchar>(total_y,total_x))
                {
                    TotalIntensityMap.at<uchar>(total_y,total_x) = intensity;
                }

                TotalMap.at<uchar>(total_y,total_x) = avg_val;
            }
        }
    }
}

void MAPTYPEGENERATE::Update2DMap(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground)
{
    int intensity;

    double x,y,z;
    double px,py,pz;

    for (int i = 0; i < cloud_ground->points.size(); ++i)
    {
        x = cloud_ground->points[i].x;
        y = cloud_ground->points[i].y;
        z = cloud_ground->points[i].z;

        px = x+pose_x;
        py = y+pose_y;
        pz = z;

        intensity = cloud_ground->points[i].intensity;

        if(intensity>30)
            intensity=30;
        if(intensity<10)
            intensity=10;
        intensity = intensity/30.0f*255;

        int total_x = (px-MapRange.min_x)/grid_size;
        int total_y = TotalMap.rows-1-(py-MapRange.min_y)/grid_size;
        int grey = int((pz+1.0)*255/(2.2+1.0));

        if(total_x>=0&&total_x<TotalMixMap.cols&&total_y>=0&&total_y<TotalMixMap.rows)
        {
            //if(grey > m_TotalMap.at<uchar>(total_y, total_x)) {
            if(1)
            {
                int avg_val = TotalMixMap.at<uchar>(total_y, total_x);
                int count_val = CountToalMap.at<uchar>(total_y, total_x);
                if(count_val >= 20) {
                    avg_val = (avg_val*(count_val - 1) + grey) / count_val;
                }
                else {
                    int sum_val = avg_val * count_val + grey;
                    count_val ++;
                    avg_val = sum_val / count_val;
                    CountToalMap.at<uchar>(total_y, total_x) = count_val;
                }

                if(intensity>TotalMixMap.at<uchar>(total_y,total_x))
                {
                    TotalMixMap.at<uchar>(total_y,total_x) = intensity;
                }
            }
        }
    }

    for (int i = 0; i < cloud_no_ground->points.size(); ++i)
    {
        x = cloud_no_ground->points[i].x;
        y = cloud_no_ground->points[i].y;
        z = cloud_no_ground->points[i].z;

        px = x+pose_x;
        py = y+pose_y;
        pz = z;

        intensity = cloud_no_ground->points[i].intensity;

        if(intensity>30)
            intensity=30;
        if(intensity<10)
            intensity=10;
        intensity = intensity/30.0f*255;

        int total_x = (px-MapRange.min_x)/grid_size;
        int total_y = TotalMap.rows-1-(py-MapRange.min_y)/grid_size;
        int grey = int((pz+1.0)*255/(2.2+1.0));

        if(total_x>=0&&total_x<TotalMixMap.cols&&total_y>=0&&total_y<TotalMixMap.rows)
        {
            //if(grey > m_TotalMap.at<uchar>(total_y, total_x)) {
            if(1)
            {
                int avg_val = TotalMixMap.at<uchar>(total_y, total_x);
                int count_val = CountToalMap.at<uchar>(total_y, total_x);
                if(count_val >= 20) {
                    avg_val = (avg_val*(count_val - 1) + grey) / count_val;
                }
                else {
                    int sum_val = avg_val * count_val + grey;
                    count_val ++;
                    avg_val = sum_val / count_val;
                    CountToalMap.at<uchar>(total_y, total_x) = count_val;
                }
                TotalMixMap.at<uchar>(total_y,total_x) = avg_val;
            }
        }
    }
}


void MAPTYPEGENERATE::WriteConfig()
{
    string date;
    time_t now = time(0);
    tm *ltm = localtime(&now);
    date = std::to_string(1900 + ltm->tm_year) + "-" +std::to_string(1 + ltm->tm_mon) + "-"+ std::to_string(ltm->tm_mday) + "-" + std::to_string(ltm->tm_hour);

    std::cout << "save config file-----------------------\n";
    std::stringstream configpath_mapdb;

    configpath_mapdb << sub_save_MaptilePath << "/config";
    std::cout << "configpath_mapdb : " << configpath_mapdb.str() << std::endl;

    if(access(configpath_mapdb.str().c_str(),0)==-1)
    {
        if(mkdir(configpath_mapdb.str().c_str(),0744)==-1)
        {
            printf("The config folder create error!\n");
        }
    }

    stringstream configstram ;
    configstram << configpath_mapdb.str() << "/config.ini";
    ofstream output(configstram.str().c_str());

    if(output.is_open()!=1)
    {
        cout << "Fail to open map position file:" << configstram.str() << endl;
        return;
    }

    output<<"##地图版本online参数\n";
    output<<"MapAdditionX= "<<setprecision(10)<<pose_x<<std::endl;
    output<<"MapAdditionY= "<<setprecision(10)<<pose_y<<std::endl;
    output<<"VoxelSize= "<<setprecision(2)<<map_voxel_size<<std::endl;
    output<<"##地图站点坐标，方便可改写\n";
    output<<"MapCreateID= "<<setprecision(10)<<"mapedit"<<std::endl;
    output<<"SationX= "<<setprecision(10)<<station_lon<<std::endl;
    output<<"SationY= "<<setprecision(10)<<station_lat<<std::endl;
    output<<"Hight= "<<setprecision(10)<<station_height<<std::endl;
    output<<"Zone= "<<setprecision(10)<<ugv_zone<<std::endl;
    output<<"Score= "<<setprecision(2)<<score<<std::endl;
    output<<"TileSize= "<<setprecision(10)<<tile_size<<std::endl;
    output<<"MapType= "<<setprecision(10)<<map_type<<std::endl;
    output<<"Date= " <<date << std::endl;
    output.close();

    printf("[WriteConfig] 具体online地图参数:basex=%lf basey=%lf vsize=%lf stationx=%lf stationy=%lf station_hight=%lf \n",
           pose_x,pose_y ,0.5, station_lon,station_lat,station_height);
}


//void MAPTYPEGENERATE::Load3dPath()
//{
//    struct dirent *ptr;
//    DIR *dir;
//    std::string mapTilePath;

//    std::vector<std::string> files2;
//    files2.clear();
//    mapTilePath = sub_save_3DPath ;

//    dir = opendir(mapTilePath.c_str());
//    while((ptr=readdir(dir))!=NULL)
//    {
//        if(ptr->d_name[0] =='.')
//            continue;
//        //        std::cout << "filename : " << ptr->d_name << std::endl;
//        files2.push_back(ptr->d_name);
//        std::stringstream filename;
//        filename << mapTilePath << "/"<< ptr->d_name ;
//        std::cout << mapTilePath<< std::endl;
//        vCloudfilename2.push_back(filename.str());
//    }
//    closedir(dir);
//}


void MAPTYPEGENERATE::SaveTileMap()
{
    std::cout << "save tile map.....................................................\n";
//    getSaveTileInfo();
//    Load3dPath();
    WriteConfig();
//    pcl::PointCloud<pcl::PointXYZI>::Ptr every_input(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI> input;
//    input.clear();

    int points_num = 0;
//    double index = 15.0f/grid_num;
    for(int i=0;i<grid_num;i++)
    {
        if(grids.at(i).cloud.size()>0)
        {
            pcl::io::savePCDFileBinaryCompressed(grids.at(i).filename, grids.at(i).cloud);
            std::cout << "Wrote " << grids.at(i).cloud.points.size() << " points to "
                      << grids.at(i).filename << "." << std::endl;
            points_num += grids.at(i).cloud.points.size();
//            usleep(200000);
        }
    }

//    auto t2 = std::chrono::system_clock::now();
//    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000.0;

//    std::cout << " Totoal points num : " << points_num << " points."
//              << "using time : " << diff << "ms" << std::endl;
//    input.clear();
    grids.clear();

}

void MAPTYPEGENERATE::Get2D3DGlobalCloud()
{
    mMapDataFrame->final_2D_map_cloud_ptr->clear();
    mMapDataFrame->final_3D_map_cloud_ptr->clear();

    if(mMapDataFrame->cloud_keyframes_v.size()==0)
    {
        std::cout << "no keyframe cloud" << std::endl;
        return;
    }

    for(size_t id=0; id<mMapDataFrame->cloud_keyframes_v.size(); id++)
    {
        pcl::PointCloud<PointType3D> transformed_2Dcloud;
        Eigen::AngleAxisf rotation_x(mMapDataFrame->cloud_keyposes_6d_ptr->points.at(id).roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf rotation_y(mMapDataFrame->cloud_keyposes_6d_ptr->points.at(id).pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rotation_z(mMapDataFrame->cloud_keyposes_6d_ptr->points.at(id).yaw, Eigen::Vector3f::UnitZ());
        Eigen::Translation3f translation(mMapDataFrame->cloud_keyposes_6d_ptr->points.at(id).x, mMapDataFrame->cloud_keyposes_6d_ptr->points.at(id).y, mMapDataFrame->cloud_keyposes_6d_ptr->points.at(id).z);
        Eigen::Matrix4f T = (translation * rotation_z * rotation_y * rotation_x).matrix();
        transformed_2Dcloud = *GetTransformPoint2DCloud(mMapDataFrame->cloud_keyframes_v[id], T);
        *mMapDataFrame->final_3D_map_cloud_ptr += *GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[id].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[id]);
        *mMapDataFrame->final_2D_map_cloud_ptr += transformed_2Dcloud;
    }
}

cv::Mat MAPTYPEGENERATE::ConvertToRainbow(const cv::Mat& scaledGray)
{
    /*
     * color    R   G   B   gray
     * red      255 0   0   255
     * orange   255 127 0   204
     * yellow   255 255 0   153
     * green    0   255 0   102
     * cyan     0   255 255 51
     * blue     0   0   255 0
     *
    */
    cv::Mat outputRainbow(scaledGray.size(), CV_8UC3);
    unsigned char grayValue;

    for (int y = 0; y < scaledGray.rows; y++)
    {
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<uchar>(y, x);
            cv::Vec3b& pixel = outputRainbow.at<cv::Vec3b>(y, x);
/*            if(grayValue>=0)
            {
                pixel[0] = 0;
                pixel[1] = 0;
                pixel[2] = 0;
            }
            else */if (grayValue <= 51)
            {
                pixel[0] = 0;
                pixel[1] = grayValue * 5;
                pixel[2] = 0;
            }
            else if (grayValue <= 102)
            {
                grayValue -= 51;
                pixel[0] = 255 - grayValue * 5;
                pixel[1] = 255;
                pixel[2] = 0;

                if(pixel[2]==0 && pixel[1] == 255)
                {
                    pixel[0] = 128;
                    pixel[1] = 128;
                    pixel[2] = 128;
                }
            }
            else if (grayValue <= 153)
            {
                grayValue -= 102;
                pixel[0] = 0;
                pixel[1] = 255;
                pixel[2] = grayValue * 5;
                if(pixel[2]<180 )
                {
                    pixel[0] = 128;
                    pixel[1] = 128;
                    pixel[2] = 128;
                }
            }
            else if (grayValue <= 204)
            {
                grayValue -= 153;
                pixel[0] = 0;
                pixel[1] = 255 - static_cast<unsigned char>(grayValue * 128.0 / 51 + 0.5);
                pixel[2] = 255;
            }
            else if (grayValue <= 255)
            {
                grayValue -= 204;
                pixel[0] = 0;
                pixel[1] = 127 - static_cast<unsigned char>(grayValue * 127.0 / 51 + 0.5);
                pixel[2] = 255;
            }
        }
    }
    return outputRainbow;
    return outputRainbow;
}

void MAPTYPEGENERATE::save_submap()
{
    float filter_value = save_config_->save_submap_filter_;
    std::string file_name = std::to_string(min_x_) + "_" + std::to_string(min_y_) + "_" + std::to_string(max_x_) + "_" + std::to_string(max_y_);

    if(fabs(filter_value) < 1e-6)
    {
        if(!submap_->empty())
        {
            pcl::io::savePCDFileBinaryCompressed(sub_save_MaptilePath + "/submap/" + file_name, *submap_);
        }
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filter_point(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> filter;
        filter.setLeafSize(filter_value, filter_value, filter_value);
        filter.setInputCloud(submap_);
        filter.filter(*filter_point);

        if(!filter_point->empty())
        {
            pcl::io::savePCDFileBinaryCompressed(sub_save_MaptilePath + "/submap/" + file_name, *filter_point);
        }
    }
}
//inline pcl::PointCloud<PointType3D>::Ptr GetTransformPoint2DCloud(pcl::PointCloud<PointType3D> cloudIn, Eigen::Matrix4f Trans, float car_height)
//{
//    PointType3D p;
//    int cloudSize = cloudIn.points.size();
//    pcl::PointCloud<PointType3D>::Ptr cloudOut(new pcl::PointCloud<PointType3D>);
//    double x,y,z;
//    double height_ref;
//    for (int i = 0; i < cloudSize; ++i)
//    {
//        x = cloudIn.points[i].x;
//        y = cloudIn.points[i].y;
//        z = cloudIn.points[i].z+ugv_height;
//        int intensity = int(cloudIn.points[i].intensity);

//        if(sqrt(x*x+y*y)>30)
//            continue;
//        if(x!=0 || y!=0 )
//        {
//            if(x>-2.0 && x<2.0 && y<=3.0 && y>=-3)
//                continue;
//            if(z<-1.0 || z>2.2)    //if(z<-1 || z>15)
//                continue;
//            if(y<-15 || y>15)   continue;
//            p.x = Trans(0,0)*x +Trans(0,1)*y +Trans(0,2)*z +Trans(0,3);
//            p.y = Trans(1,0)*x +Trans(1,1)*y +Trans(1,2)*z +Trans(1,3);
//            height_ref = Trans(2,0)*x +Trans(2,1)*y +Trans(2,2)*z +Trans(2,3);
//            p.x = p.x*100.0f ;  //cm
//            p.y = p.y*100.0f ;
//            p.z = z;
//            //            p.z = height_ref  - Trans(2,3);
//            p.intensity = intensity;
//            cloudOut->push_back(p);
//        }
//    }
//    return cloudOut;
//}

