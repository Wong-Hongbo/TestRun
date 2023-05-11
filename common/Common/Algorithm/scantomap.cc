/******************************************************************************
 * Copyright(c)2017 XingShen Technology of GuangZhou Ltd .
 * All rights reserved
 *
 * @file       scantomap.cc
 * @brief
 *
 * @author     Yu Hui Liang
 * @date       2021/03/10
 * @history
 *****************************************************************************/
#include "scantomap.hh"
#include "Common/Covproject.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <gtsam/geometry/OrientedPlane3.h>
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include <vtkLine.h>

#include <QDir>

double voxel_size[2] = {1, 1};
Eigen::Quaterniond q_curr(1, 0, 0, 0);
Eigen::Vector3d t_curr(0, 0, 0);
Eigen::Quaterniond delta_q(1, 0, 0, 0);
Eigen::Vector3d delta_t(0, 0, 0);
string keyframe_path;

//lyx add <<--
std::mutex g_keyframe_ds;
//-->>

inline void get_random_color(float &r, float &g, float &b, float range_max) //range_max example 1,255...
{
    r = range_max * (rand() / (1.0 + RAND_MAX));
    g = range_max * (rand() / (1.0 + RAND_MAX));
    b = range_max * (rand() / (1.0 + RAND_MAX));
}

SCANTOMAP::SCANTOMAP(MAPDATAFRAME* MapDataFrame)
    :mMapDataFrame(MapDataFrame),/*,
      registration(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>()),*/
      cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>()), cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>()),
      surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>()), surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>()),
      laserCloud(new pcl::PointCloud<pcl::PointXYZI>())
{
    matchingscore = new MatchingScore();
    ReadMapEditConfig();

    registration = select_registration_method();

    downSizeFilterICP.setLeafSize(map_voxel_size, map_voxel_size, map_voxel_size);
    downSizeFilterGlobalMapKeyPoses.setLeafSize(1, 1, 1);

//    threadbackEnd = boost::thread(&SCANTOMAP::BackEndThread,this);

//    threadBundleAdustmentOdom = boost::thread(&SCANTOMAP::BundleAdustmentOdomThread, this);

//    if(b_show3d)
//        thread3dMap = boost::thread(&SCANTOMAP::Visualize3DMapThread,this);

    string home_path = getenv("HOME");

    std::cout << "home path : " << home_path << std::endl;

    std::stringstream filename;
    filename << home_path << "/.MappingEdit/tmp" ;
    keyframe_path = filename.str();
//    sub_save_MaptilePath = filename.str();
//    std::cout << "mapdb path  : " << sub_save_MaptilePath << std::endl;
    if(access(filename.str().c_str(),0)==-1)
    {
        if(mkdir(filename.str().c_str(),0744)==-1)
        {
            printf("tmp folder create error!\n");
        }
    }

//    std::stringstream rm_mod;
//    rm_mod << "rm -rf " <<  keyframe_path << "/*";
//    system(rm_mod.str().c_str());

    QDir dir(QString::fromStdString(keyframe_path));
    dir.removeRecursively();

    if(access(filename.str().c_str(),0)==-1)
    {
        if(mkdir(filename.str().c_str(),0744)==-1)
        {
            printf("tmp folder create error!\n");
        }
    }


    mMapDataFrame->keyframe_path = keyframe_path;
    mMapDataFrame->interactive_mode =1;
    ResetSystem();

    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(GRAVITY);

    // Tightly-coupled system is really serious about noise. So, I recommend to set covariance higher.
    // Also, z-axis covariance is much lower because gravity is more confident.
    // for safety tightly coupled. If you regulate the value, you may get more accurate results.
    p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise*10, 2); // acc white noise in continuous
    p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise*10, 2); // gyro white noise in continuous
    p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1, 2); // error committed in integrating position from velocities
    p->integrationCovariance(2,2) = pow(0.1, 2);
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3); // m/s
    priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
    noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
    imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    threadfrontEnd = boost::thread(&SCANTOMAP::FrontEndThread,this);
    threadglobalMap = boost::thread(&SCANTOMAP::VisualizeGlobalMapThread, this);
    threadloopdetect = boost::thread(&SCANTOMAP::LoopClosureThread, this);
}


void SCANTOMAP::ResetSystem()
{
    cache_cloud_keyframes.clear();
    cache_cloud_keyframes_raw.clear();
    cache_ndt_pose.clear();
    cache_plane.clear();
    save_count = 0;
    _previous_pose.x = 0.0;
    _previous_pose.y = 0.0;
    _previous_pose.z = 0.0;
    _previous_pose.roll = 0.0;
    _previous_pose.pitch = 0.0;
    _previous_pose.yaw = 0.0;
    offsetlocalpose.x = 0;
    offsetlocalpose.y = 0;
    offsetlocalpose.z = 0;
    offsetlocalpose.yaw = 0;
    offsetlocalpose.roll = 0;
    offsetlocalpose.pitch = 0;
    recent_keyframes_.clear();

    _pre_keypose.x = 0;
    _pre_keypose.y = 0;
    _pre_keypose.z = 0;
    _pre_keypose.yaw = 0;
    _pre_keypose.roll = 0;
    _pre_keypose.pitch = 0;
    //    pre_keypose= _previous_pose;
    loop_closure_enabled_ =  true;
    _initial_scan_loaded =false;
    bGetZoneInit = false;
    //    aLoopIsClosed = false;
    _loop_closed = false;
    bReset = false;
    bBA_init = false;
    guess_pose = _current_pose = _previous_gps = _pre_keypose;
    sensor_height = 0;
    tilt_deg = 0;
    height_clip_range = 1.0;
    floor_pts_thresh = 1000;
    floor_normal_thresh =10;
    use_normal_filtering = true;
    normal_filter_thresh = 20;
    history_fitness_score_ = 0.2;
    match_score_range = 1.0;

    //    initguess_matrix = Eigen::Identity();
    //    std::cout << "initguess_matrix : " << initguess_matrix << std::endl;
    AllocateMemory();
    mMapDataFrame->AllMapDataFrameMemory();
    mMapDataFrame->LoopClosingMapFrameMemory();
}

void SCANTOMAP::AllocateMemory()
{
    input_lidar.reset(new pcl::PointCloud<pcl::PointXYZI>());
    plane_lidar.reset(new pcl::PointCloud<pcl::PointXYZI>());
    scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    filter_scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    trans_filter_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    _target_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType3D>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointType6D>());
    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>()) ;
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    globalMapKeyFrames.reset(new pcl::PointCloud<pcl::PointXYZI>());
    globalMapKeyFramesDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
    global2dMapKeyFrames.reset(new pcl::PointCloud<pcl::PointXYZI>());
    global2dMapKeyFramesDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
    global2dMapKeyPoses.reset(new pcl::PointCloud<pcl::PointXYZI>());
    global2dMapKeyPosesDS.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cureKeyframeCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    prevKeyframeCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
//    keyposes_3d_ptr.reset(new pcl::PointCloud<PointType3D>);
//    keyposes_2d_ptr.reset(new pcl::PointCloud<PointType2D>);
//    keyposes_6d_ptr.reset(new pcl::PointCloud<PointType6D>);
}


SCANTOMAP::~SCANTOMAP()
{
    threadfrontEnd.interrupt();
    threadfrontEnd.join();
    thread3dMap.interrupt();
    thread3dMap.join();
    threadloopdetect.interrupt();
    threadloopdetect.join();
    threadglobalMap.interrupt();
    threadglobalMap.join();
    threadbackEnd.interrupt();
    threadbackEnd.join();
    threadBundleAdustmentOdom.interrupt();
    threadBundleAdustmentOdom.join();
}


void SCANTOMAP::VisualizeGlobalMapThread()
{
    while(1)
    {
        boost::this_thread::interruption_point();
        //        if(show_map_update==0)
        //            return;
        publishGlobalMap();
        usleep(1000000);
    }
}

void SCANTOMAP::Visualize3DMapThread()
{
    //    signal(SIGINT, thread_2dmashow_loop_quit);
    pc_viewer = InitViewer();
    while(1)
    {
        boost::this_thread::interruption_point();
        if(!pc_viewer->wasStopped())
        {
//            std::cout <<"viewer 3d " << std::endl;
            Publish3dMap();
            //        usleep(300000);
            pc_viewer->spinOnce(100);
        }
    }
}

void SCANTOMAP::LoopClosureThread()
{
    //    signal(SIGINT, thread_loopdetect_quit);
    if (!loop_closure_enabled_)
    {
        return;
    }

    while(1)
    {
        boost::this_thread::interruption_point();
        //        if(show_map_update==0)
        //            return;
        PerformLoopClosure();

        //        if(b_sc==1)
        //        {

        //            performLoopClosure_sc();

        //        }
        //        LoopClosure();
        usleep(100000);
    }
}

void SCANTOMAP::BundleAdustmentOdomThread()
{
    while(1)
    {
        boost::this_thread::interruption_point();
        BundleAdustmentOdom();
        usleep(20000);
    }
}


void SCANTOMAP::BackEndThread()
{
    //    if (!loop_closure_enabled_)
    //    {
    //        return;
    //    }

    while(1)
    {
        boost::this_thread::interruption_point();
        KeyFrameUpdater();
        usleep(20000);
    }
}


void SCANTOMAP::Publish3dMap()
{
    //    if(bStart || !bStop)
    {
        if (mMapDataFrame->cloud_keyposes_3d_ptr->points.empty() || bReset==1 /*|| show_map_update==0 */)
            return;
        //        if(height_Map)
        Update3DshowMap();
        //        if(grid_Map)
        //            update2DGridMap();
    }
}


void SCANTOMAP::publishGlobalMap()
{
    if (mMapDataFrame->cloud_keyposes_3d_ptr->points.empty())
        return;

    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    pointSearchIndGlobalMap.clear();
    pointSearchSqDisGlobalMap.clear();

    mtx.lock();
    kdtreeGlobalMap->setInputCloud(mMapDataFrame->cloud_keyposes_3d_ptr);
    // 通过KDTree进行最近邻搜索
    kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, globalMapVisualizationSearchRadius,
                                  pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    pcl::PointCloud<pcl::PointXYZI> globalMapKeyPoses;

    globalMapKeyPoses.clear();

    for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
    {
        globalMapKeyPoses.points.push_back(mMapDataFrame->cloud_keyposes_3d_ptr->points[pointSearchIndGlobalMap[i]]);
    }

    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses.makeShared());

    downSizeFilterGlobalMapKeyPoses.filter(globalMapKeyPoses);

    global2dMapKeyFrames->clear();
    globalMapKeyFrames->clear();
    RPYpose pose;
    Eigen::Matrix4f T;
    for (int i = 0; i < globalMapKeyPoses.points.size(); ++i)
    {
        int thisKeyInd = (int)globalMapKeyPoses.points[i].intensity;

        pose.x =  mMapDataFrame->cloud_keyposes_6d_ptr->points[thisKeyInd].x;
        pose.y =  mMapDataFrame->cloud_keyposes_6d_ptr->points[thisKeyInd].y;
        pose.z =  mMapDataFrame->cloud_keyposes_6d_ptr->points[thisKeyInd].z;

        pose.roll =  mMapDataFrame->cloud_keyposes_6d_ptr->points[thisKeyInd].roll;
        pose.pitch =  mMapDataFrame->cloud_keyposes_6d_ptr->points[thisKeyInd].pitch;
        pose.yaw =  mMapDataFrame->cloud_keyposes_6d_ptr->points[thisKeyInd].yaw;
        RPYposeToMatrix(pose,T);

        //lyx modify<<--
        //*global2dMapKeyFrames += *GetTransformPoint2DCloud(mMapDataFrame->cloud_keyframes_v[thisKeyInd], T);
        //*globalMapKeyFrames += *GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[thisKeyInd].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[thisKeyInd]);
        //==>>
        if(mMapDataFrame->cloud_keyframes_v.size() > 0)
        {
            *global2dMapKeyFrames += *GetTransformPoint2DCloud(mMapDataFrame->cloud_keyframes_v[thisKeyInd], T);
            *globalMapKeyFrames += *GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[thisKeyInd].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[thisKeyInd]);
        }
        //-->>

        //lyx add <<--
        g_keyframe_ds.lock();
        //-->>

        globalMapKeyFramesDS->width = 1;
        globalMapKeyFramesDS->height = globalMapKeyFramesDS->points.size();
        g_keyframe_ds.unlock();

#ifdef SHOW3D_
        *globalMapKeyFrames += *transformPointCloud(cloud_keyframes_[thisKeyInd], cloud_keyposes_6d_->points[thisKeyInd]);
#endif
    }
#ifdef SHOW3D_
    // 对globalMapKeyFrames进行下采样
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

    globalMapKeyFramesDS->width = 1;
    globalMapKeyFramesDS->height = globalMapKeyFramesDS->points.size();
#endif

    //show_lck.lock();
    global2dMapKeyFramesDS->clear();
    *global2dMapKeyFramesDS = *global2dMapKeyFrames;

    //lyx add <<--
    g_keyframe_ds.lock();
    //-->>
    globalMapKeyFramesDS->clear();
    *globalMapKeyFramesDS = *globalMapKeyFrames;
    g_keyframe_ds.unlock();
    //show_lck.unlock();

    //    emit output_point_cloud(globalMapKeyFramesDS);
    //    emit output_kf_pose(mMapDataFrame->cloud_keyposes_3d_ptr);

    //std::cerr << "aaaaaaaaaa" << std::endl;
#ifdef SHOW3D_
    globalMapKeyFrames->clear();
#endif
}

void SCANTOMAP::add_lines_to_viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                                    pcl::PointCloud<PointT>::Ptr pts_1,  pcl::PointCloud<PointT>::Ptr pts_2,
                                    const std::string lines_name,
                                    const std::vector<unsigned int> &red, const std::vector<unsigned int> &green, const std::vector<unsigned int> &blue,
                                    int down_rate)

{
    //reference: https://vtk.org/Wiki/VTK/Examples/Cxx/GeometricObjects/ColoredLines

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> line;
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();

    int line_num = std::min(pts_1->points.size(), pts_2->points.size());
    //    std::cout << "line_num : " << line_num << std::endl;
    for (int i = 0; i < line_num; i++)
    {
        points->InsertNextPoint(pts_1->points[i].data);
        points->InsertNextPoint(pts_2->points[i].data);
    }

    //    std::cout << "line_num 2 : " << line_num << std::endl;

    // Add the points to the dataset
    polyData->SetPoints(points);

    colors->SetNumberOfComponents(3);

    //    std::cout << "line_num 3 : " << line_num << std::endl;


    unsigned char rgb[3];

    for (int i = 0; i < line_num; i++)
    {
        //        if (i == 0)
        {

            line = vtkSmartPointer<vtkLine>::New();
            line->GetPointIds()->SetNumberOfIds(2);
            line->GetPointIds()->SetId(0, 2 * i);
            line->GetPointIds()->SetId(1, 2 * i + 1);

            cells->InsertNextCell(line);
            rgb[0] = red[i];
            rgb[1] = green[i];
            rgb[2] = blue[i];

#if VTK_MAJOR_VERSION < 7
            colors->InsertNextTupleValue(rgb);
#else
            colors->InsertNextTypedTuple(rgb);
#endif

        }
    }

    //    std::cout << "line_num 4 : " << line_num << std::endl;


    // Add the lines to the dataset
    polyData->SetLines(cells);
    // Add the color
    polyData->GetCellData()->SetScalars(colors);

    viewer->addModelFromPolyData(polyData, lines_name);

    //    std::cout << "line_num 5 : " << line_num << std::endl;

}


void SCANTOMAP::Update3DshowMap()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr map2dcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarcloud_init(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr lidarcloud_rotation(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    //    pc_viewer->removeCoordinateSystem();
    map2dcloud->clear();
    show_lck.lock();
    *map2dcloud  = *globalMapKeyFramesDS ;
    show_lck.unlock();

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Vector4f plane_coffes;
    tmp1->clear();
    tmp2->clear();
    plane_cloud->clear();
    //        tmp->clear();
    lidarcloud->clear();
    lidarcloud_init->clear();
    lidarcloud_init->clear();
    show_lidar_lck.lock();
    *tmp1 = *input_lidar;
    *plane_cloud = *plane_lidar;
    plane_coffes = coffes;

    show_lidar_lck.unlock();

    //Draw pose
    //    pcl::PointXYZ pt_c(_current_pose.x, _current_pose.y, _current_pose.z);

    //    std::cout << "tmp1 size : " << tmp1->size() << std::endl;
    pcl::transformPointCloud(*tmp1,*lidarcloud,final_matrix);
    pcl::transformPointCloud(*tmp1,*lidarcloud_init,initguess_matrix);
    pcl::transformPointCloud(*tmp1,*lidarcloud_rotation,delta_lp_matrix);

    //    boost::mutex::scoped_lock update_lock(update_mutex_);
    //    pc_viewer->addCoordinateSystem(0,_current_pose.x,_current_pose.y,_current_pose.z,0);

    //    pc_viewer->addSphere(pt_c, 0.5, -255, -255, -255, 0);
    //    pc_viewer->setCameraPosition(0,0,_current_pose.z,0,0,0,0);

    pc_viewer->removeAllPointClouds();
    pc_viewer->removeShape("sphere");
    pc_viewer->removeShape("edges");
//    pc_viewer->removeShape("plane");
    pc_viewer->removePointCloud("nodes");

//    std::cout << "plane_coffes : " << plane_coffes << std::endl;

//    pcl::ModelCoefficients plane;
//    plane.values.push_back(plane_coffes[0]);
//    plane.values.push_back(plane_coffes[1]);
//    plane.values.push_back(plane_coffes[2]);
//    plane.values.push_back(plane_coffes[3]);
    double p_x,p_y,p_z;
    p_x = mMapDataFrame->cloud_keyposes_3d_ptr->back().x;
    p_y = mMapDataFrame->cloud_keyposes_3d_ptr->back().y;
    p_z = mMapDataFrame->cloud_keyposes_3d_ptr->back().z;
//    pc_viewer->addPlane(plane/*,p_x,p_y,p_z*/);

    float line_width = 2.0;
    float node_size = 7.0;
    double font_color = 1.0;

    int edge_count = cons.size();

    std::vector<unsigned int> edge_r(edge_count), edge_g(edge_count), edge_b(edge_count);

    pcl::PointCloud<PointT>::Ptr pc_nodes_1(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr pc_nodes_2(new pcl::PointCloud<PointT>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nodes(new pcl::PointCloud<pcl::PointXYZRGB>());

    for(int i=0;i<edge_count; i++)
    {
        float r, g, b;

        PointT ptc1;

        ptc1.x = mMapDataFrame->cloud_keyposes_3d_ptr->points[cons[i].id1].x;
        ptc1.y = mMapDataFrame->cloud_keyposes_3d_ptr->points[cons[i].id1].y;
        ptc1.z = mMapDataFrame->cloud_keyposes_3d_ptr->points[cons[i].id1].z;

        pc_nodes_1->points.push_back(ptc1);

        pcl::PointXYZRGB ptc1_rgb;
        ptc1_rgb.x = ptc1.x;
        ptc1_rgb.y = ptc1.y;
        ptc1_rgb.z = ptc1.z;
        get_random_color(r, g, b, 255);
        ptc1_rgb.r = r;
        ptc1_rgb.g = g;
        ptc1_rgb.b = b;
        nodes->points.push_back(ptc1_rgb);

        PointT ptc2;
        //        ptc2.x = cons[i].node2.x;
        //        ptc2.y = cons[i].node2.y;
        //        ptc2.z = cons[i].node2.z;

        ptc2.x = mMapDataFrame->cloud_keyposes_3d_ptr->points[cons[i].id2].x;
        ptc2.y = mMapDataFrame->cloud_keyposes_3d_ptr->points[cons[i].id2].y;
        ptc2.z = mMapDataFrame->cloud_keyposes_3d_ptr->points[cons[i].id2].z;


        pc_nodes_2->points.push_back(ptc2);

        pcl::PointXYZRGB ptc2_rgb;
        ptc2_rgb.x = ptc2.x;
        ptc2_rgb.y = ptc2.y;
        ptc2_rgb.z = ptc2.z;
        get_random_color(r, g, b, 255);
        ptc2_rgb.r = r;
        ptc2_rgb.g = g;
        ptc2_rgb.b = b;
        nodes->points.push_back(ptc2_rgb);

        if(cons[i].type=="reg")
        {
            edge_r[i] = 255;
            edge_g[i] = 0;
            edge_b[i] = 255;
        }
        else if(cons[i].type=="loop")
        {
            edge_r[i] = 255;
            edge_g[i] = 255;
            edge_b[i] = 0;
        }
    }

    //    std::cout << "map2dcloud size4 : " << map2dcloud->size() << std::endl;

    add_lines_to_viewer(pc_viewer, pc_nodes_1, pc_nodes_2, "edges", edge_r, edge_g, edge_b, 0);

    //    std::cout << "map2dcloud size5 : " << map2dcloud->size() << std::endl;

    pc_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, "edges");

    pc_viewer->addPointCloud(nodes, "nodes");

    pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, node_size, "nodes");


    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> lo_pose_color(mMapDataFrame->cloud_keyposes_3d_ptr, font_color * 255, font_color * 255, font_color * 255);
    pc_viewer->addPointCloud(mMapDataFrame->cloud_keyposes_3d_ptr, lo_pose_color, "lo_pose");
    pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "lo_pose");

    pcl::PointXYZ pt_c(mMapDataFrame->cloud_keyposes_6d_ptr->back().x, mMapDataFrame->cloud_keyposes_6d_ptr->back().y, mMapDataFrame->cloud_keyposes_6d_ptr->back().z);
    pc_viewer->addSphere(pt_c, 0.5, 1, 1, 1,"sphere");

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> single_color(map2dcloud, "intensity");
    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color (map2dcloud, 0, 255, 255);

    if (!pc_viewer->updatePointCloud<PointT>(map2dcloud, single_color, "map"))
    {
        pc_viewer->addPointCloud<PointT>(map2dcloud, single_color, "map");
        pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "map");
    }

//        pcl::visualization::PointCloudColorHandlerGenericField<PointT> single_color(lidarcloud, "z");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> lidar_h (lidarcloud, 0, 255, 0);

        if (!pc_viewer->updatePointCloud<PointT>(lidarcloud, lidar_h, "lidar"))
        {
            pc_viewer->addPointCloud<PointT>(lidarcloud, lidar_h, "lidar");
            pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                        4, "lidar");
        }
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> lidar_init_h (plane_cloud, 0, 255, 0);

//    if (!pc_viewer->updatePointCloud<PointT>(plane_cloud, lidar_init_h, "lidar_init"))
//    {
//        pc_viewer->addPointCloud<PointT>(plane_cloud, lidar_init_h, "lidar_init");
//        pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
//                                                    5, "lidar_init");
//    }

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> SCANTOMAP::InitViewer()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
    cloud_->width = 5;
    cloud_->height = 1;
    cloud_->is_dense = false;
    cloud_->points.resize(cloud_->width * cloud_->height);

    for (size_t i=0; i<cloud_->points.size(); ++i)
    {
        cloud_->points[i].x = 0;
        cloud_->points[i].y = 0;
        cloud_->points[i].z = 0;
        cloud_->points[i].intensity = 0;
    }
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    //    viewer->addPointCloud<PointT>(cloud_, "sample cloud");
    //    viewer->addCoordinateSystem(1.0);
    //    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "sample cloud");
    //    viewer->initCameraParameters();

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> single_color(cloud_, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud_, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    return (viewer);
}

void SCANTOMAP::FrontEndThread()
{
    while(1)
    {
        boost::this_thread::interruption_point();
        Run();
        usleep(10000);
    }
}

void SCANTOMAP::Matching()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
    registration->setInputSource(filter_scan_ptr);
    registration->setInputTarget(_target_points);
//    pcl::io::savePCDFileBinaryCompressed("target_map.pcd",*_target_points);
    registration->align(*aligned,initguess_matrix);
    t_localizer  =registration->getFinalTransformation();
}

bool SCANTOMAP::Process(HDLADARDATA_MSG* lidar_data)
{
    auto StartTime=  std::chrono::system_clock::now();

    lidardata =  lidar_data;
    time_LaserInfoCur =  lidardata->LocalPose.time;
    ExtractSurroundKeyframes();
    GetInsAndLpInfo();
    GetLidarCloudAndFilter();
    RPYposeToMatrix(guess_pose, initguess_matrix);

    pcl::PointCloud<pcl::PointXYZI> transformed_scan;
    transformed_scan.clear();

    //std::cout << "filter_scan_ptr : " << filter_scan_ptr->size() << std::endl;
    pcl::transformPointCloud(*filter_scan_ptr,transformed_scan,initguess_matrix);

    if (!_initial_scan_loaded)
    {
        //        std::cout << "init map" << std::endl;
        *_target_points += transformed_scan; // target map is first cloud
        //        _prev_scan_time = curr_scan_time;
        _initial_scan_loaded = true;
        return false;
    }

    Matching();

    MatrixToRPYpose(t_localizer, localizer_pose);
    _current_pose.x = localizer_pose.x;
    _current_pose.y = localizer_pose.y;
    _current_pose.z = localizer_pose.z;
    _current_pose.yaw = localizer_pose.yaw;
    _current_pose.time = time_LaserInfoCur;
    _current_pose.roll = localizer_pose.roll;
    _current_pose.pitch = localizer_pose.pitch;

    RPYposeToMatrix(_current_pose, final_matrix);

    pcl::transformPointCloud(*filter_scan_ptr,transformed_scan,final_matrix);

    input_lidar->clear();
    *input_lidar = *filter_scan_ptr;

    //    std::cout << "1111 " << std::endl;
    {
        std::lock_guard<std::mutex> lock(mtx);
        HumanCorrectPose();
        CorrectPose();

        if(KeyFrameUpdater())
        {
            mMapDataFrame->cloud_keyframes_v.push_back(*filter_scan_ptr);
            //if(scan_ptr->size()>0)
                pcl::io::savePCDFileBinaryCompressed(keyframe_path + "/" + std::to_string(save_count) + ".pcd",*scan_ptr);  //保存当前原始关键幀结果
            save_count++;
        }
    }
    _previous_pose = _current_pose;
    auto EndTime=  std::chrono::system_clock::now();
    float process_time =  std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime ).count() / 1000.0;
    time = process_time;

}

bool SCANTOMAP::SaveFrame()
{
    if (mMapDataFrame->cloud_keyposes_3d_ptr->points.empty())
        return true;
    //    if(cache_ndt_pose.size()>0)
    {
        Eigen::Affine3f transStart = pclPointToAffine3f(mMapDataFrame->cloud_keyposes_6d_ptr->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(_current_pose.x, _current_pose.y, _current_pose.z,
                _current_pose.roll, _current_pose.pitch, _current_pose.yaw);

        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
        //std::cout <<"save frame ==== " <<"x : " << x << "y : " << y << "z : " << z << "roll : " << roll << "pitch : " << pitch << "yaw : " << yaw << std::endl;

        //        std::cout <<"save frame ==== " <<"ndt pose : " << cache_ndt_pose[0].x << "y : " << cache_ndt_pose[0].y << "z : " << cache_ndt_pose[0].z << "roll : " << cache_ndt_pose[0].roll << "pitch : " << cache_ndt_pose[0].pitch << "yaw : " << cache_ndt_pose[0].yaw << std::endl;

        float surroundingkeyframeAddingAngleThreshold =0.2;
        float surroundingkeyframeAddingDistThreshold = 1.0;
        float dis = sqrt(x*x + y*y + z*z);
        //std::cout << " save dis : " << dis  << std::endl;
        if (/*abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
                                            abs(pitch) < surroundingkeyframeAddingAngleThreshold &&*/
//                abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
                dis < surroundingkeyframeAddingDistThreshold)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

}


void SCANTOMAP::GlobalBatchOptimizer()
{
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2DoglegParams dogleg_param;
    dogleg_param.setVerbose(false);
    parameters.setOptimizationParams(dogleg_param);
    this->isam.reset(new gtsam::ISAM2(parameters));
    this->initial_estimate_.reset(new gtsam::Values);
    this->gtSAMgraph_.reset(new gtsam::NonlinearFactorGraph());
    isam_current_estimate_.clear();

    for(int i=0; i<mMapDataFrame->cloud_keyposes_6d_ptr->size();i++)
    {
        if(i==0)
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter

            this->gtSAMgraph_->add(PriorFactor<Pose3>(gtsam::symbol('x',0), Pose3(Rot3::RzRyRx(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].roll, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].pitch, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].yaw),
                                                                                  Point3(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].x, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].y, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].z)), priorNoise));

            this->initial_estimate_->insert(gtsam::symbol('x',0), Pose3(Rot3::RzRyRx(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].roll, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].pitch, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].yaw),
                                                                        Point3(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].x, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].y, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].z)));
        }
        else
        {
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

            gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(mMapDataFrame->cloud_keyposes_6d_ptr->points[i-1].roll, mMapDataFrame->cloud_keyposes_6d_ptr->points[i-1].pitch, mMapDataFrame->cloud_keyposes_6d_ptr->points[i-1].yaw),
                    Point3(mMapDataFrame->cloud_keyposes_6d_ptr->points[i-1].x, mMapDataFrame->cloud_keyposes_6d_ptr->points[i-1].y, mMapDataFrame->cloud_keyposes_6d_ptr->points[i-1].z));
            gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].roll, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].pitch , mMapDataFrame->cloud_keyposes_6d_ptr->points[i].yaw),
                                         Point3(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].x, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].y, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].z));

            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1,1,1,1,1,1).finished()); // rad*rad, meter*meter

            this->gtSAMgraph_->add(PriorFactor<Pose3>(gtsam::symbol('x',i), Pose3(Rot3::RzRyRx(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].roll, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].pitch, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].yaw),
                                                                                  Point3(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].x, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].y, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].z)), priorNoise));

            this->gtSAMgraph_->add(BetweenFactor<Pose3>(gtsam::symbol('x',i-1), gtsam::symbol('x',i) , pose_from.between(pose_to), odometryNoise));

            this->initial_estimate_->insert(gtsam::symbol('x',i), Pose3(Rot3::RzRyRx(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].roll, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].pitch, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].yaw),
                                                                        Point3(mMapDataFrame->cloud_keyposes_6d_ptr->points[i].x, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].y, mMapDataFrame->cloud_keyposes_6d_ptr->points[i].z)));
        }
    }

    gtsam::Vector vector6(6);

    vector6 << 1, 1, 1, 1, 1, 1;

    noiseModel::Diagonal::shared_ptr constraint_noise_ = noiseModel::Diagonal::Variances(vector6);

    if(mMapDataFrame->cloud_loop_index->size()>0)
    {
        for(int i=0;i<mMapDataFrame->cloud_loop_index->size();i++)
        {
            gtsam::Pose3 loop_pose = gtsam::Pose3(Rot3::RzRyRx(mMapDataFrame->cloud_loop_index->points[i].roll, mMapDataFrame->cloud_loop_index->points[i].pitch, mMapDataFrame->cloud_loop_index->points[i].yaw),
                                                  Point3(mMapDataFrame->cloud_loop_index->points[i].x, mMapDataFrame->cloud_loop_index->points[i].y,
                                                         mMapDataFrame->cloud_loop_index->points[i].z));

            //                std::cout <<   i <<  "  " << cloud_loop_index->points[i].loop_from <<  " " <<  cloud_loop_index->points[i].loop_to << std::endl;

            this->gtSAMgraph_->add(BetweenFactor<Pose3>(gtsam::symbol('x',mMapDataFrame->cloud_loop_index->points[i].loop_from),
                                                        gtsam::symbol('x',mMapDataFrame->cloud_loop_index->points[i].loop_to), loop_pose, constraint_noise_));
        }
    }

    this->isam->update(*this->gtSAMgraph_,*this->initial_estimate_);
    this->isam->update();
    this->isam->update();
    this->isam->update();
    this->isam->update();
    this->isam->update();
    this->gtSAMgraph_->resize(0);
    this->initial_estimate_->clear();
    isam_current_estimate_ = this->isam->calculateEstimate();

}

bool SCANTOMAP::KeyFrameUpdater()
{
    auto start_1 = std::chrono::system_clock::now();

    double roll,pitch,yaw;
    roll =_current_pose.roll;
    pitch = _current_pose.pitch;
    yaw = _current_pose.yaw;

    currentRobotPosPoint.x = _current_pose.x ;
    currentRobotPosPoint.y = _current_pose.y ;
    currentRobotPosPoint.z = _current_pose.z ;

    bool saveThisKeyFrame = true;
    if (sqrt((previousRobotPosPoint.x-currentRobotPosPoint.x)*(previousRobotPosPoint.x-currentRobotPosPoint.x)
             +(previousRobotPosPoint.y-currentRobotPosPoint.y)*(previousRobotPosPoint.y-currentRobotPosPoint.y)
             +(previousRobotPosPoint.z-currentRobotPosPoint.z)*(previousRobotPosPoint.z-currentRobotPosPoint.z)) >1.0){
        saveThisKeyFrame = false;
        previousRobotPosPoint = currentRobotPosPoint;
    }

    if (saveThisKeyFrame == false && !mMapDataFrame->cloud_keyposes_3d_ptr->points.empty())
        return false;

    if (SaveFrame() == false )
    {
        return false;
    }

    AddOdomFactor();
    //    addGPSFactorNew();
    //    addGPSFactorNew2();
    //    AddLoopFactor();

    // update iSAM
    auto start_isam_1 = std::chrono::system_clock::now();
    this->isam->update(*this->gtSAMgraph_,*this->initial_estimate_);
    //    this->isam->update();
    auto end_isam_1 = std::chrono::system_clock::now();
    auto diff1 = std::chrono::duration_cast<std::chrono::microseconds>(end_isam_1 - start_isam_1).count() / 1000.0;
//        std::cout << " isam update 1 time : " << diff1 << std::endl;

    auto start_isam_2 = std::chrono::system_clock::now();
    this->isam->update();
    auto end_isam_2  = std::chrono::system_clock::now();

    auto diff_update = std::chrono::duration_cast<std::chrono::microseconds>(end_isam_2 - start_isam_2).count() / 1000.0;
//        std::cout << " isam update time 2 : " << diff_update<< std::endl;

    //     update multiple-times till converge

    //    if (_loop_closed )
    //    {
    //        this->isam->update();
    //        this->isam->update();
    //        this->isam->update();
    //        this->isam->update();
    //        this->isam->update();
    //    }

    auto start_clear = std::chrono::system_clock::now();
    this->gtSAMgraph_->resize(0);
    this->initial_estimate_->clear();

    auto end_clear = std::chrono::system_clock::now();
    auto diff2 = std::chrono::duration_cast<std::chrono::microseconds>(end_clear - start_clear).count() / 1000.0;
//        std::cout << " isam clear time : " << diff2<< std::endl;

    auto start_estimate_ = std::chrono::system_clock::now();
    isam_current_estimate_ = this->isam->calculateEstimate();

    auto end_estimate_ = std::chrono::system_clock::now();
    auto diff3 = std::chrono::duration_cast<std::chrono::microseconds>(end_estimate_ - start_estimate_).count() / 1000.0;
//            std::cout << " calculateEstimate  time : " << diff3<< std::endl;
//            std::cout << "this->isam_current_estimate_.size()  : " << this->isam_current_estimate_.size()  << std::endl;

    pcl::PointXYZI this_pose_3d;
    pcl::PointXY this_pose_2d;
    PointXYZIRPYT this_pose_6d;
    PointTGPS this_pose_rtk_dev;

    Pose3 latest_estimate;

    int size = mMapDataFrame->cloud_keyposes_3d_ptr->points.size();
//    std::cout << "size : " << size << std::endl;
//    latest_estimate = isam_current_estimate_.at<Pose3>(gtsam::symbol('x',(this->isam_current_estimate_.size() - 1)));
    latest_estimate = isam_current_estimate_.at<Pose3>(gtsam::symbol('x',size));

    this_pose_2d.x = this_pose_6d.x = this_pose_3d.x = latest_estimate.translation().x();
    this_pose_2d.y = this_pose_6d.y = this_pose_3d.y = latest_estimate.translation().y();
    this_pose_6d.z = this_pose_3d.z = latest_estimate.translation().z();
    this_pose_6d.intensity = this_pose_3d.intensity = mMapDataFrame->cloud_keyposes_3d_ptr->points.size();
    this_pose_6d.roll = latest_estimate.rotation().roll();
    this_pose_6d.pitch = latest_estimate.rotation().pitch();
    this_pose_6d.yaw  = latest_estimate.rotation().yaw();
    this_pose_6d.time = _current_pose.time;
    this_pose_rtk_dev.x = _gps_pose.x;
    this_pose_rtk_dev.y = _gps_pose.y;
    this_pose_rtk_dev.z = _gps_pose.z;
    this_pose_rtk_dev.yaw = _gps_pose.yaw;
    this_pose_rtk_dev.roll = _gps_pose.roll;
    this_pose_rtk_dev.pitch = _gps_pose.pitch;
    this_pose_rtk_dev.intensity = mMapDataFrame->cloud_keyposes_3d_ptr->points.size();
    this_pose_rtk_dev.lon_dev = _gps_pose.lon_dev;
    this_pose_rtk_dev.lat_dev = _gps_pose.lat_dev;
    this_pose_rtk_dev.time = _gps_pose.time;
    this_pose_rtk_dev.lon = _gps_pose.lon;
    this_pose_rtk_dev.lat = _gps_pose.lat;
    this_pose_rtk_dev.height_dev = _gps_pose.height_dev;

    pose_covariance = this->isam->marginalCovariance(gtsam::symbol('x',size));

    mMapDataFrame->cloud_keyposes_2d_ptr->push_back(this_pose_2d);
    mMapDataFrame->cloud_keyposes_3d_ptr->push_back(this_pose_3d);
    mMapDataFrame->cloud_keyposes_6d_ptr->push_back(this_pose_6d);

//    std::cout << "scanto map cloud key pose size : " << mMapDataFrame->cloud_keyposes_6d_ptr->size() << std::endl;
//    std::cout << "kf x : " << this_pose_6d.x << std::endl;
//    std::cout << "kf y : " << this_pose_6d.y << std::endl;
//    std::cout << "kf z : " << this_pose_6d.z << std::endl;
//    std::cout << "kf roll : " << this_pose_6d.roll << std::endl;
//    std::cout << "kf pitch : " << this_pose_6d.pitch << std::endl;
//    std::cout << "kf yaw : " << this_pose_6d.yaw << std::endl;
    auto end_1 = std::chrono::system_clock::now();
    auto diff4 = std::chrono::duration_cast<std::chrono::microseconds>(end_1 - start_1).count() / 1000.0;
    //    std::cout << " total   time : " << diff4<< std::endl;

    //    std::cout << " keyframe size  : " << mMapDataFrame->cloud_keyposes_6d_ptr->size()<< std::endl;

    if (mMapDataFrame->cloud_keyposes_3d_ptr->points.size() > 1)
    {
        _pre_keypose.x = this_pose_3d.x;
        _pre_keypose.y = this_pose_3d.y;
        _pre_keypose.z = this_pose_3d.z;
        _pre_keypose.roll = latest_estimate.rotation().roll();
        _pre_keypose.pitch = latest_estimate.rotation().pitch();
        _pre_keypose.yaw  = latest_estimate.rotation().yaw();
        _pre_keypose.time = _current_pose.time;
    }
}

void SCANTOMAP::AddImuFactor()
{
    if(mMapDataFrame->cloud_keyposes_3d_ptr -> points.empty())
    {
        while (!imu_buffer_.empty())
        {
            if ((imu_buffer_.front()->data.time) < (lidar_buffer_.front()->LocalPose.time))
            {
                lastImuT_opt =  imu_buffer_.front()->data.time;
                imu_buffer_.pop_front();
            }
            else
                break;
        }
        // initial velocity
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(gtsam::symbol('v',0), prevVel_, priorVelNoise);
        gtSAMgraph_->add(priorVel);
        // initial bias
        prevBias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(gtsam::symbol('b',0), prevBias_, priorBiasNoise);
        gtSAMgraph_->add(priorBias);
        // add values
        initial_estimate_->insert(gtsam::symbol('v',0), prevVel_);
        initial_estimate_->insert(gtsam::symbol('b',0), prevBias_);

        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
    }
    else
    {
        while (!imu_buffer_.empty())
        {
            // pop and integrate imu data that is between two optimizations
            double imu_time = imu_buffer_.front()->data.time;

            double acc_y = (double)imu_buffer_.front()->data.acc_x/IMU_MAG_GAIN;
            double acc_x = -(double)imu_buffer_.front()->data.acc_y/IMU_MAG_GAIN;
            double acc_z = (double)imu_buffer_.front()->data.acc_z/IMU_MAG_GAIN;
            double gyro_y = (double)imu_buffer_.front()->data.rot_x/IMU_MAG_GAIN*deg_to_rad;
            double gyro_x = -(double)imu_buffer_.front()->data.rot_y/IMU_MAG_GAIN*deg_to_rad;
            double gyro_z = (double)imu_buffer_.front()->data.rot_z/IMU_MAG_GAIN*deg_to_rad;

            if ( imu_time < (lidar_buffer_.front()->LocalPose.time))
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imu_time - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(acc_x, acc_y, acc_z),
                        gtsam::Vector3(gyro_x,gyro_y,gyro_z), dt);

                lastImuT_opt = imu_time;
                imu_buffer_.pop_front();
            }
            else
                break;
        }

        int imu_pre_size = mMapDataFrame->cloud_keyposes_3d_ptr->size()-1;
        int imu_cur_size = mMapDataFrame->cloud_keyposes_3d_ptr->size();

        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(gtsam::symbol('x',imu_pre_size), gtsam::symbol('v',imu_pre_size),
                                    gtsam::symbol('x',imu_cur_size), gtsam::symbol('v',imu_cur_size),
                                    gtsam::symbol('b',imu_pre_size), preint_imu);
        gtSAMgraph_->add(imu_factor);

        // add imu bias between factor
        gtSAMgraph_->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(gtsam::symbol('b',imu_pre_size), gtsam::symbol('b',imu_cur_size), gtsam::imuBias::ConstantBias(),
                        gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        initial_estimate_->insert(gtsam::symbol('v',imu_cur_size), propState_.v());
        initial_estimate_->insert(gtsam::symbol('b',imu_cur_size), prevBias_);

    }

}

void SCANTOMAP::AddOdomFactor()
{
    double roll,pitch,yaw;
    roll =_current_pose.roll;
    pitch = _current_pose.pitch;
    yaw = _current_pose.yaw;
    if(mMapDataFrame->cloud_keyposes_3d_ptr->points.empty())
    {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;

        this->isam.reset(new gtsam::ISAM2(parameters));
        this->initial_estimate_.reset(new gtsam::Values);
        this->gtSAMgraph_.reset(new gtsam::NonlinearFactorGraph());

        noiseModel::Diagonal::shared_ptr priorNoise;
        gtsam::Vector vector6_prior(6);
        vector6_prior << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8;
//        vector6_prior << 0.5,0.5,0.5,0.1,0.1,0.1;
        priorNoise = noiseModel::Diagonal::Variances(vector6_prior); // rad*rad, meter*meter

        this->gtSAMgraph_->add(PriorFactor<Pose3>(gtsam::symbol('x',0), Pose3(Rot3::RzRyRx(_current_pose.roll, _current_pose.pitch, _current_pose.yaw),
                Point3(_current_pose.x, _current_pose.y, _current_pose.z)), priorNoise));
        this->initial_estimate_->insert(gtsam::symbol('x',0),
                                        Pose3(Rot3::RzRyRx(_current_pose.roll, _current_pose.pitch, _current_pose.yaw),
                Point3(_current_pose.x, _current_pose.y, _current_pose.z)));
    }
    else
    {
        gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(_pre_keypose.roll, _pre_keypose.pitch, _pre_keypose.yaw), Point3(_pre_keypose.x, _pre_keypose.y, _pre_keypose.z));
        gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(_current_pose.roll, _current_pose.pitch, _current_pose.yaw), Point3(_current_pose.x, _current_pose.y, _current_pose.z));
        noiseModel::Diagonal::shared_ptr odometryNoise;

        odometryNoise    = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

        this->gtSAMgraph_->add(BetweenFactor<Pose3>(gtsam::symbol('x',(mMapDataFrame->cloud_keyposes_3d_ptr->points.size()- 1)),
                                                    gtsam::symbol('x',(mMapDataFrame->cloud_keyposes_3d_ptr->points.size())), pose_from.between(pose_to), odometryNoise));

        pose_graph edge;
        edge.id1 = mMapDataFrame->cloud_keyposes_3d_ptr->points.size()- 1;
        edge.id2 = mMapDataFrame->cloud_keyposes_3d_ptr->points.size();
        edge.type  = "reg";
        cons.push_back(edge);

        int size = mMapDataFrame->cloud_keyposes_3d_ptr->points.size();

        Pointlooppose pt;
        pt.x = pose_from.between(pose_to).translation().x();
        pt.y = pose_from.between(pose_to).translation().y();
        pt.z = pose_from.between(pose_to).translation().z();
        pt.roll = pose_from.between(pose_to).rotation().roll();
        pt.pitch = pose_from.between(pose_to).rotation().pitch();
        pt.yaw = pose_from.between(pose_to).rotation().yaw();

        pt.loop_from = mMapDataFrame->cloud_keyposes_3d_ptr->points.size()-1;
        pt.loop_to = mMapDataFrame->cloud_keyposes_3d_ptr->points.size();

//        mMapDataFrame->cloud_loop_index->push_back(pt);

        this->initial_estimate_->insert(gtsam::symbol('x',size),
                                        Pose3(Rot3::RzRyRx(_current_pose.roll, _current_pose.pitch, _current_pose.yaw),
                Point3(_current_pose.x, _current_pose.y, _current_pose.z)));
        mMapDataFrame->cloud_pose_graph_ptr->push_back(pt);
    }
}

//void SCANTOMAP::AddGpsFactor(const RPYpose gnss_pose)
//{
//    mBackEnd->addGnssConstraint(gnss_pose);
//}



void SCANTOMAP::AddLoopFactor()
{
    if (loop_Index_Queue.empty())
        return;

    for (int i = 0; i < (int)loop_Index_Queue.size(); ++i)
    {

        int indexFrom = loop_Index_Queue[i].first;
        int indexTo = loop_Index_Queue[i].second;
        RPYpose pose_from = loop_Pose_from_Queue[i];
        RPYpose pose_to = loop_Pose_to_Queue[i];
        gtsam::Pose3 poseBetween = loop_Pose_Queue[i];

        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loop_Noise_Queue[i];
        this->gtSAMgraph_->add(BetweenFactor<Pose3>(gtsam::symbol('x',indexFrom), gtsam::symbol('x',indexTo), poseBetween, noiseBetween));

        pose_graph lc_edge;
        lc_edge.id1 = indexFrom;
        lc_edge.id2 = indexTo;
        lc_edge.type = "loop";
        cons.push_back(lc_edge);
        std::cout << "add loop constration.." << std::endl;
        //         mBackEnd->addLoopConstraint(indexFrom, indexTo,pose_from, pose_to);
    }

    loop_Index_Queue.clear();
    loop_Pose_from_Queue.clear();
    loop_Pose_to_Queue.clear();
    loop_Noise_Queue.clear();
    _loop_closed = true;
}


void SCANTOMAP::HumanCorrectPose()
{
    if(mMapDataFrame->human_correct_pose)
    {
        GlobalBatchOptimizer();

        recent_keyframes_.clear();
        cache_ndt_pose.clear();
        cache_cloud_keyframes.clear();
        cache_cloud_keyframes_raw.clear();
        cache_plane.clear();

        int latest_history_frame_id_ = mMapDataFrame->cloud_keyposes_6d_ptr->size()-1;

       previousRobotPosPoint.x =  _current_pose.x =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].x;
       previousRobotPosPoint.y =  _current_pose.y =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].y;
       previousRobotPosPoint.z =  _current_pose.z =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].z;
        _current_pose.roll =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].roll;
       _current_pose.pitch =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].pitch;
        _current_pose.yaw =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].yaw;

        std::cout << "human corrent_current_pose x : " << _current_pose.x << std::endl;
        std::cout << "human corrent_current_pose y : " << _current_pose.y << std::endl;
        std::cout << "human corrent_current_pose z : " << _current_pose.z << std::endl;
        std::cout << "human corrent_current_pose roll : " << _current_pose.roll << std::endl;
        std::cout << "human corrent_current_pose pitch : " << _current_pose.pitch << std::endl;
        std::cout << "human corrent_current_pose yaw : " << _current_pose.yaw << std::endl;

        _pre_keypose = _previous_pose = _current_pose;
        mMapDataFrame->human_correct_pose =false;
    }
}

void SCANTOMAP::CorrectPose()
{

    if(_loop_closed )
    {
        recent_keyframes_.clear();
        cache_ndt_pose.clear();
        cache_cloud_keyframes.clear();
        cache_cloud_keyframes_raw.clear();
        cache_plane.clear();

        Eigen::Matrix4f keyposeMatrix ,matchInitguess;
        RPYpose keypose;

        int latest_history_frame_id_ = mMapDataFrame->cloud_keyposes_6d_ptr->size()-1;

        keypose.x =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].x;
        keypose.y =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].y;
        keypose.z =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].z;
        keypose.roll =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].roll;
        keypose.pitch =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].pitch;
        keypose.yaw =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].yaw;
        RPYposeToMatrix(keypose, keyposeMatrix);
        RPYposeToMatrix(_current_pose, matchInitguess);
        //        RRYposeToMatrix(_previous_pose,matchInitguess);
        Eigen::Matrix4f RT ;
        RT = matchInitguess*keyposeMatrix.inverse();
        RPYpose rt_pose;
        MatrixToRPYpose(RT,rt_pose);

        int num_poses = isam_current_estimate_.size();

        for (int i = 0; i < num_poses; ++i)
        {
            mMapDataFrame->cloud_keyposes_2d_ptr->points[i].x =  mMapDataFrame->cloud_keyposes_6d_ptr->points[i].x = mMapDataFrame->cloud_keyposes_3d_ptr->points[i].x = this->isam_current_estimate_.at<Pose3>(gtsam::symbol('x',i)).translation().x();
            mMapDataFrame->cloud_keyposes_2d_ptr->points[i].y =  mMapDataFrame->cloud_keyposes_6d_ptr->points[i].y = mMapDataFrame->cloud_keyposes_3d_ptr->points[i].y = this->isam_current_estimate_.at<Pose3>(gtsam::symbol('x',i)).translation().y();
            mMapDataFrame->cloud_keyposes_6d_ptr->points[i].z = mMapDataFrame->cloud_keyposes_3d_ptr->points[i].z = this->isam_current_estimate_.at<Pose3>(gtsam::symbol('x',i)).translation().z();
            mMapDataFrame->cloud_keyposes_6d_ptr->points[i].roll = this->isam_current_estimate_.at<Pose3>(gtsam::symbol('x',i)).rotation().roll();
            mMapDataFrame->cloud_keyposes_6d_ptr->points[i].pitch = this->isam_current_estimate_.at<Pose3>(gtsam::symbol('x',i)).rotation().pitch();
            mMapDataFrame->cloud_keyposes_6d_ptr->points[i].yaw = this->isam_current_estimate_.at<Pose3>(gtsam::symbol('x',i)).rotation().yaw();
            //            Eigen::Matrix4f pose =
            //                    this->isam_current_estimate_.at<gtsam::Pose3>(gtsam::symbol('x',i)).matrix().cast<float>();
            //            //            view_graph_.AddVertex(i,pose);
        }

        keypose.x =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].x;
        keypose.y =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].y;
        keypose.z =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].z;
        keypose.roll =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].roll;
        keypose.pitch =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].pitch;
        keypose.yaw =  mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_history_frame_id_].yaw;
        RPYposeToMatrix(keypose, keyposeMatrix);
        matchInitguess= RT*keyposeMatrix;
        MatrixToRPYpose(matchInitguess, _current_pose);

        _pre_keypose.x =  mMapDataFrame->cloud_keyposes_6d_ptr->points[num_poses-1].x;
        _pre_keypose.y =  mMapDataFrame->cloud_keyposes_6d_ptr->points[num_poses-1].y;
        _pre_keypose.z =  mMapDataFrame->cloud_keyposes_6d_ptr->points[num_poses-1].z;
        _pre_keypose.roll =  mMapDataFrame->cloud_keyposes_6d_ptr->points[num_poses-1].roll;
        _pre_keypose.pitch =  mMapDataFrame->cloud_keyposes_6d_ptr->points[num_poses-1].pitch;
        _pre_keypose.yaw =  mMapDataFrame->cloud_keyposes_6d_ptr->points[num_poses-1].yaw;

        std::cout << "corrent_current_pose x : " << _current_pose.x << std::endl;
        std::cout << "corrent_current_pose y : " << _current_pose.y << std::endl;
        std::cout << "corrent_current_pose z : " << _current_pose.z << std::endl;
        std::cout << "corrent_current_pose roll : " << _current_pose.roll << std::endl;
        std::cout << "corrent_current_pose pitch : " << _current_pose.pitch << std::endl;
        std::cout << "corrent_current_pose yaw : " << _current_pose.yaw << std::endl;

        std::cout << "this is correct pose finsh " << std::endl;
        pcl::io::savePCDFileBinaryCompressed("correct_key_pose.pcd",*mMapDataFrame->cloud_keyposes_6d_ptr);
        _previous_pose = _current_pose;
        _loop_closed =false;
    }
}

void SCANTOMAP::GetInsAndLpInfo()
{
    _gps_pose.z = lidardata->Position.height/100.0f;
    _gps_pose.time = lidardata->Position.gps_millisecond;

    _gps_pose.yaw = lidardata->Position.azimuth*M_PI/18000.0f - M_PI/2;
    _gps_pose.lon  = lidardata->Position.llhPos[0]/*/1000000.0f*/;
    _gps_pose.lat  = lidardata->Position.llhPos[1]/*/1000000.0f*/;
    _gps_pose.lon = _gps_pose.lon/1000000.0f;
    _gps_pose.lat = _gps_pose.lat/1000000.0f;

    lon_dev = lidardata->Position.drPos[0];
    lat_dev = lidardata->Position.drPos[1];


    if(_gps_pose.lon>0 && _gps_pose.lat>0 &&(!gps_state))
    {
        if(!bGetZoneInit)
        {
            if(_gps_pose.lon < 0)
                _gps_pose.lon += 360;
            ugv_zone = (int)(_gps_pose.lon/6) + 1;
            bGetZoneInit = true;
        }
    }

    double gauss_x,gauss_y;
    if(bGetZoneInit)
        GaussProjCal(_gps_pose.lon,_gps_pose.lat,&gauss_x,&gauss_y,ugv_zone);
    else
        GaussProjCal(_gps_pose.lon,_gps_pose.lat,&gauss_x,&gauss_y);
    _gps_pose.x = gauss_x;
    _gps_pose.y = gauss_y;

    _gps_pose.roll = 0;
    _gps_pose.pitch = 0;
    //    show_gps_lon_dev = _gps_pose.lon_dev = lidardata->Position.drPos[0];
    //    show_gps_lat_dev = _gps_pose.lat_dev = lidardata->Position.drPos[1];
    _gps_pose.height_dev = lidardata->Position.drHeight;
    gps_state = lidardata->Position.positionStatus;

    double curr_scan_time = lidardata->LocalPose.time;
    lp_time = curr_scan_time;

    RPYpose current_localpose;
    current_localpose.x = lidardata->LocalPose.dr_x;
    current_localpose.y = lidardata->LocalPose.dr_y;
    current_localpose.z = lidardata->LocalPose.dr_z;
    current_localpose.yaw = (lidardata->LocalPose.dr_heading)*M_PI/18000.0f;
    current_localpose.roll = lidardata->LocalPose.dr_roll*M_PI/18000.0f;
    current_localpose.pitch =  lidardata->LocalPose.dr_pitch*M_PI/18000.0f;
    current_localpose.time = lidardata->LocalPose.time;

    //    Eigen::Matrix4f current_lp_matrix,previous_lp_matrix ;

    double avg_speed = (lidardata->LocalPose.lr_speed + lidardata->LocalPose.rr_speed)/2;
    //std::cout << "avg_speed : "<< avg_speed << std::endl;

    static RPYpose previous_localpose =current_localpose;

    offsetlocalpose.x = (current_localpose.x -previous_localpose.x)/100.0f;
    offsetlocalpose.y = (current_localpose.y -previous_localpose.y)/100.0f;
    offsetlocalpose.z = (current_localpose.z -previous_localpose.z)/100.0f;
    offsetlocalpose.yaw = (current_localpose.yaw -previous_localpose.yaw);
    offsetlocalpose.roll = (current_localpose.roll -previous_localpose.roll);
    offsetlocalpose.pitch = (current_localpose.pitch -previous_localpose.pitch);

    RPYposeToMatrix(current_localpose,current_lp_matrix);
    RPYposeToMatrix(previous_localpose,previous_lp_matrix);
    Eigen::Matrix4f delta_lp_matrix =  current_lp_matrix * previous_lp_matrix.inverse();

    RPYpose delta_lp_rpy;

    MatrixToRPYpose(delta_lp_matrix,delta_lp_rpy);

    offsetlocalpose.time = current_localpose.time -previous_localpose.time;

    double distance = sqrt((offsetlocalpose.x)*(offsetlocalpose.x)+(offsetlocalpose.y)*(offsetlocalpose.y))/**1.1*/;
    //std::cout <<"dis : "<< distance << std::endl;

    double deltaX=0;
    double deltaY=0;

    if(avg_speed>0)
    {
        deltaX = distance*cos(_previous_pose.yaw + M_PI/2); // 小车 localpose
        deltaY = distance*sin(_previous_pose.yaw + M_PI/2);
    }

    if(avg_speed<0) //backward
    {
        deltaX = -distance*cos(_previous_pose.yaw + M_PI/2); // 小车 localpose
        deltaY = -distance*sin(_previous_pose.yaw + M_PI/2);
    }


//    std::cout << "_previous_pose x : " << _previous_pose.x << std::endl;
//    std::cout << "_previous_pose y : " << _previous_pose.y << std::endl;
//    std::cout << "_previous_pose z : " << _previous_pose.z << std::endl;
//    std::cout << "_previous_pose roll : " << _previous_pose.roll << std::endl;
//    std::cout << "_previous_pose pitch : " << _previous_pose.pitch << std::endl;
//    std::cout << "_previous_pose yaw : " << _previous_pose.yaw << std::endl;


    guess_pose = _previous_pose;
    guess_pose.x = _previous_pose.x + deltaX;
    guess_pose.y = _previous_pose.y + deltaY;
    guess_pose.z = _previous_pose.z;

    guess_pose.roll = _previous_pose.roll;
    guess_pose.pitch = _previous_pose.pitch;
    guess_pose.yaw = _previous_pose.yaw + offsetlocalpose.yaw;

    previous_localpose =current_localpose;
}

void SCANTOMAP::LoopFindNearKeyframes(pcl::PointCloud<pcl::PointXYZI>::Ptr& nearKeyframes, const int& key,const int& losest_key, const int& searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();

    if(searchNum==0)
    {
        height_last = copy_cloudKeyPoses6D->points[key].z;
        height_history = copy_cloudKeyPoses6D->points[losest_key].z;
        diff_height = height_last - height_history;
        std::cout << "diff_height : " << diff_height << std::endl;

        RPYpose pose_trans;
        pose_trans.roll = copy_cloudKeyPoses6D->points[key].roll;
        pose_trans.pitch = copy_cloudKeyPoses6D->points[key].pitch;
        pose_trans.yaw = copy_cloudKeyPoses6D->points[key].yaw;
        pose_trans.x = copy_cloudKeyPoses6D->points[key].x ;
        pose_trans.y =  copy_cloudKeyPoses6D->points[key].y ;
        pose_trans.z =  copy_cloudKeyPoses6D->points[key].z - diff_height;

        Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
        RPYposeToMatrix(pose_trans,this_transformation);

        pcl::transformPointCloud(mMapDataFrame->cloud_keyframes_v[key], *nearKeyframes, this_transformation);
    }
    else
    {
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[keyNear].makeShared(), copy_cloudKeyPoses6D->points[keyNear]);
        }
    }


    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

void SCANTOMAP::PerformLoopClosure()
{
    if (mMapDataFrame->cloud_keyposes_3d_ptr->points.empty() == true)
        return;
    cureKeyframeCloud->clear();
    prevKeyframeCloud->clear();

    mtx.lock();
    *copy_cloudKeyPoses3D = *mMapDataFrame->cloud_keyposes_3d_ptr;
    *copy_cloudKeyPoses6D = *mMapDataFrame->cloud_keyposes_6d_ptr;
    mtx.unlock();

    // find keys
    int loopKeyCur;
    int loopKeyPre;
    //    if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
    if (DetectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
        return;

    std::cout << "loop key cur : " << loopKeyCur << " loop key pre : " << loopKeyPre << std::endl;
    // extract cloud
    {
        LoopFindNearKeyframes(cureKeyframeCloud, loopKeyCur,loopKeyPre, 0);
        LoopFindNearKeyframes(prevKeyframeCloud, loopKeyCur,loopKeyPre, search_map_num);
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
    }

    bool has_converged=false;
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> registration_loop;
    pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
    registration_loop.setTransformationEpsilon(0.01);
    registration_loop.setResolution(2.0);
    registration_loop.setStepSize(0.5);
    registration_loop.setNeighborhoodSearchMethod(pclomp::DIRECT1);
    registration_loop.setNumThreads(1);
    registration_loop.setMaximumIterations(64);
    registration_loop.setInputTarget(prevKeyframeCloud);
    registration_loop.setInputSource(cureKeyframeCloud);
    registration_loop.align(*unused_result);
    pcl::io::savePCDFileBinaryCompressed("prevKeyframeCloud.pcd",*prevKeyframeCloud);
    pcl::io::savePCDFileBinaryCompressed("cureKeyframeCloud.pcd",*cureKeyframeCloud);

    has_converged = registration_loop.hasConverged();
    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Matrix4f correctionLidarFrame;
//    correctionLidarFrame = icp.getFinalTransformation();
    correctionLidarFrame = registration_loop.getFinalTransformation();
    // transform from world origin to wrong pose

    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_keyframe_trans(new pcl::PointCloud<pcl::PointXYZI>);

    latest_keyframe_trans->clear();

    pcl::transformPointCloud(*cureKeyframeCloud,*latest_keyframe_trans,correctionLidarFrame);
//    pcl::io::savePCDFileBinaryCompressed("");

    RPYpose rpy_pose;
    rpy_pose.x  = copy_cloudKeyPoses6D->points[loopKeyCur].x;
    rpy_pose.y  = copy_cloudKeyPoses6D->points[loopKeyCur].y;
    rpy_pose.z  = copy_cloudKeyPoses6D->points[loopKeyCur].z-diff_height;
    rpy_pose.roll  = copy_cloudKeyPoses6D->points[loopKeyCur].roll;
    rpy_pose.pitch  = copy_cloudKeyPoses6D->points[loopKeyCur].pitch;
    rpy_pose.yaw  = copy_cloudKeyPoses6D->points[loopKeyCur].yaw;

    Eigen::Matrix4f inital_guess(Eigen::Matrix4f::Identity());
    RPYposeToMatrix(rpy_pose,inital_guess);
//    Eigen::Affine3f tWrong = pclPointToAffine3f(rpy_pose);
    // transform from world origin to corrected pose
    Eigen::Matrix4f tCorrect = correctionLidarFrame * inital_guess;// pre-multiplying -> successive rotation about a fixed frame
//    pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);

    update_fitness_score(correctionLidarFrame/*.inverse()*/);
    RPYpose pose;
    MatrixToRPYpose(tCorrect,pose);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(pose.roll, pose.pitch, pose.yaw), Point3(pose.x, pose.y, pose.z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    RPYpose ryp_pose_from;
    ryp_pose_from.x = x;
    ryp_pose_from.x = y;
    ryp_pose_from.x = z;

    RPYpose ryp_pose_to;
    ryp_pose_to.x = copy_cloudKeyPoses6D->points[loopKeyPre].x;
    ryp_pose_to.x = copy_cloudKeyPoses6D->points[loopKeyPre].y;
    ryp_pose_to.x = copy_cloudKeyPoses6D->points[loopKeyPre].z;

    gtsam::Vector Vector6(6);
//    float noiseScore = icp.getFitnessScore();
//    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
      Vector6 << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001;
//    Vector6 << 1,1,1,1,1,1;

    processMatchingScore(latest_keyframe_trans,prevKeyframeCloud);

    if((has_converged == false ) || (fitness_score_loop>history_fitness_score_) || (match_score<0.4) )
    {
        printf("ndt loop cannot closed\n");
        printf("检测到闭环点，但是没有满足约束条件:\n,"
               "has_converged=%d, fitness_score_loop=%f,match_score=%f,latest keyframe size=%d,闭环点id=%d， 当前点id=%d \n",has_converged,fitness_score_loop,match_score,cureKeyframeCloud->size(),loopKeyPre,loopKeyCur);
        return ;
    }

    noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    printf("开始拉！！！！闭环满足约束条件==================================>>>>>>\n "
           "fitness_score=%f,闭环点id=%d， 当前点id=%d\n",fitness_score_loop, loopKeyPre, loopKeyCur);

//    int size = copy_cloudKeyPoses3D->size();
    std::lock_guard<std::mutex> lock(mtx);
//    Pose3 latest_estimate=isam_current_estimate_.at<Pose3>(gtsam::symbol('x',size);
    //    std::cout<<"before closure  x:"<<latest_estimate.translation().x()<<"  y:"<<latest_estimate.translation().y()<<"   z"<<latest_estimate.translation().z()<<std::endl;

    this->gtSAMgraph_->add(BetweenFactor<Pose3>(gtsam::symbol('x',loopKeyCur), gtsam::symbol('x',loopKeyPre), poseFrom.between(poseTo), constraintNoise));
    //    this->gtSAMgraph_->add(BetweenFactor<Pose3>(gtsam::symbol('x',latest_history_frame_id_), gtsam::symbol('x',closest_history_frame_id_), pose_from.between(pose_to), robustNoiseModel));

    isam->update(*this->gtSAMgraph_);
    isam->update();
    isam->update();
    isam->update();
    isam->update();
    isam->update();

    this->gtSAMgraph_->resize(0);
    this->isam_current_estimate_ = this->isam->calculateEstimate();

    pose_graph lc_edge;
    lc_edge.id1 = loopKeyCur;
    lc_edge.id2 = loopKeyPre;
    lc_edge.type = "loop";
    cons.push_back(lc_edge);
    std::cout << "add loop constration.." << std::endl;

    Pointlooppose pt;
    pt.loop_from = loopKeyCur;
    pt.loop_to = loopKeyPre;

    //    RPYpose pose_output;
    //    Eigen::Matrix4f loop_matrix;
    auto loop_matrix = poseFrom.between(poseTo);
    //    loop_matrix.x();
    //    MatrixToRPYpose(loop_matrix,pose_output);
    pt.x = loop_matrix.translation().x();
    pt.y = loop_matrix.translation().y();
    pt.z = loop_matrix.translation().z();
    pt.roll = loop_matrix.rotation().roll();
    pt.pitch = loop_matrix.rotation().pitch();
    pt.yaw = loop_matrix.rotation().yaw();

    mMapDataFrame->cloud_pose_graph_ptr->push_back(pt);
    mMapDataFrame->cloud_loop_index->push_back(pt);
    _loop_closed = true;

    // add loop constriant
    loop_IndexContainer[loopKeyCur] = loopKeyPre;

    std::cout << "start correct ..." << std::endl;
}

bool SCANTOMAP::DetectLoopClosureDistance(int *latestID, int *closestID)
{
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    double x_last = copy_cloudKeyPoses3D->points[loopKeyCur].x ;
    double y_last = copy_cloudKeyPoses3D->points[loopKeyCur].y ;

    // check loop constraint added before
    auto it = loop_IndexContainer.find(loopKeyCur);
    if (it != loop_IndexContainer.end())
        return false;

    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), search_loop_radius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    //    double  historyKeyframeSearchTimeDiff = 30;
    //    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    //    {
    //        int id = pointSearchIndLoop[i];

    //        if (abs(copy_cloudKeyPoses6D->points[id].time - time_LaserInfoCur) > historyKeyframeSearchTimeDiff)
    //        {
    //            std::cout << "copy_cloudKeyPoses6D->points[id].time - time_LaserInfoCur : " << copy_cloudKeyPoses6D->points[id].time - time_LaserInfoCur << std::endl;
    //            loopKeyPre = id;
    //            break;
    //        }
    //    }

    double min_dist=100000000.0;
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        //        std::cout << "latest_history_frame_id_ - search_idx_[i] :  " << latest_history_frame_id_ - search_idx_[i] << std::endl;
        if (fabs(loopKeyCur - pointSearchIndLoop[i]) > (search_loop_radius))
        {
            double x_closed = copy_cloudKeyPoses3D->points[pointSearchIndLoop[i]].x;
            double y_closed = copy_cloudKeyPoses3D->points[pointSearchIndLoop[i]].y;
            double dist=sqrt((x_last-x_closed)*(x_last-x_closed)+(y_last-y_closed)*(y_last-y_closed));
            if(dist<min_dist)
            {
                loopKeyPre = pointSearchIndLoop[i];
                min_dist=dist;
            }
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}


void SCANTOMAP::ExtractSurroundKeyframes()
{
    if (mMapDataFrame->cloud_keyframes_v.empty())
    {
        return;
    }

    bool target_updated = false;
    if (loop_closure_enabled_)
    {
        if (recent_keyframes_.size() < search_map_num)
        {
            recent_keyframes_.clear();
            for (int i = mMapDataFrame->cloud_keyposes_3d_ptr->points.size() - 1; i >= 0; --i)
            {
                int this_key_id = int(mMapDataFrame->cloud_keyposes_3d_ptr->points[i].intensity);
                pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                tf_cloud = GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[this_key_id].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[this_key_id]);
                recent_keyframes_.push_back(tf_cloud);
                if (recent_keyframes_.size() >= search_map_num)
                {
                    break;
                }
            }
            target_updated = true;
        }
        else
        {
            static int latest_frame_id = mMapDataFrame->cloud_keyframes_v.size() - 1;
            if (latest_frame_id != mMapDataFrame->cloud_keyframes_v.size() - 1)
            {
                latest_frame_id = mMapDataFrame->cloud_keyframes_v.size() - 1;
                recent_keyframes_.pop_back();
                pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                tf_cloud = GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[latest_frame_id].makeShared(), mMapDataFrame->cloud_keyposes_6d_ptr->points[latest_frame_id]);
                recent_keyframes_.push_front(tf_cloud);
                target_updated = true;
            }
        }
    }

    if (target_updated)
    {
        _target_points->clear();
        //        std::cout << "target size : "  <<recent_keyframes_.size() << std::endl;
        for (auto keyframe : recent_keyframes_)
        {
            *_target_points += *keyframe;
            //            std::cout << "target pointcloud size : " << _target_points.size() << std::endl;
        }

        //        ds_target_points.setInputCloud(_target_points);
        //        ds_target_points.filter(*_target_points);
    }
}


void SCANTOMAP::ScanVoxelFilterDynamic()
{
    filter_scan_ptr->clear();
    auto filter_start = std::chrono::system_clock::now();
    // std::cout << "use_dynamic leaf size : " << use_dynamic_leaf_size << std::endl;
    if (use_dynatic_filter == 0)
    {
        scan_voxel_size = 0.4;
    }

    //    mutex_scan.lock();
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));

    // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)

    if (scan_voxel_size < 0.1)
    {
        scan_voxel_size = 0.1;
    }

    if (min_voxel_size < 0.1)
    {
        min_voxel_size = 0.1;
    }

    if (use_dynatic_filter == 1)
    {
        if (scan_voxel_size < min_voxel_size)
        {
            scan_voxel_size = min_voxel_size;
        }
        if (scan_voxel_size > max_voxel_size)
        {
            scan_voxel_size = max_voxel_size;
        }
    }

    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(scan_voxel_size, scan_voxel_size, scan_voxel_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filter_scan_ptr);

    if (use_dynatic_filter == 1)
    {
        if (filter_scan_ptr->points.size() < min_pointsize)
        {
            for (; scan_voxel_size > min_voxel_size; scan_voxel_size -= voxel_size_step)
            {
                pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
                voxel_grid_filter.setLeafSize(scan_voxel_size, scan_voxel_size, scan_voxel_size);
                voxel_grid_filter.setInputCloud(scan_ptr);
                voxel_grid_filter.filter(*filter_scan_ptr);
                if (filter_scan_ptr->points.size() > min_pointsize)
                {
                    break;
                }
            }
        }
        else if (filter_scan_ptr->points.size() > max_pointsize)
        {
            for (; scan_voxel_size < max_voxel_size; scan_voxel_size += voxel_size_step)
            {
                pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
                voxel_grid_filter.setLeafSize(scan_voxel_size, scan_voxel_size, scan_voxel_size);
                voxel_grid_filter.setInputCloud(scan_ptr);
                voxel_grid_filter.filter(*filter_scan_ptr);
                if (filter_scan_ptr->points.size() < max_pointsize)
                {
                    break;
                }
            }
        }
    }
    //    std::cout << "filter_scan size : " << filter_scan_ptr->size() << std::endl;
    auto filter_end = std::chrono::system_clock::now();
    auto df_time  =std::chrono::duration_cast<std::chrono::microseconds>(filter_end - filter_start).count() / 1000.0;
}


void SCANTOMAP::GetLidarCloudAndFilter()
{
    double x,y,z,intensity;
    float _min_scan_range = 1 ;
    float _max_scan_range = 80;
    scan_ptr->clear();
    filter_scan_ptr->clear();
    laserCloud->clear();
    pcl::PointXYZI pt,pt_loam;
    int iter=0;
    for(int i=0;i< lidardata->ringNum;i++)
    {
        for(int j=0;j< PACKETNUM*6;j++)
        {
            x = lidardata->HDData[i][j].x/100.0f;
            y = lidardata->HDData[i][j].y/100.0f;
            z = lidardata->HDData[i][j].z/100.0f + ugv_height;

            float range = sqrt(pow(x, 2.0) + pow(y, 2.0));

            if(x>-1.5 && x<1.5 && y<=2.3 && y>=-2.5)
                continue;

            //            if(z<-2.79)
            //                continue;

            if (1.0 < range && range < 60.0)
            {
                intensity = int(lidardata->HDData[i][j].Intensity);
                if(x!=0 || y!=0 || z!=0)
                {
                    if(use_backpack)
                    {
                        pt.x = current_lp_matrix(0,0) *x + current_lp_matrix(0,1) *y + current_lp_matrix(0,2) *z;
                        pt.y = current_lp_matrix(1,0) *x + current_lp_matrix(1,1) *y + current_lp_matrix(1,2) *z;
                        pt.z = current_lp_matrix(2,0) *x + current_lp_matrix(2,1) *y + current_lp_matrix(2,2) *z;
                    }
                    else
                    {
                        pt.x = x;
                        pt.y = y;
                        pt.z = z;
                    }

//                    // loam cooradinate
//                    pt_loam.x = -x;
//                    pt_loam.z = y;
//                    pt_loam.y = z;
//                    pt_loam.intensity = i;
                    pt.intensity = intensity;
                    scan_ptr->push_back(pt);
//                    laserCloud->push_back(pt_loam);
                    iter++;
                }
            }
        }
//        scanEndInd[i] = iter - 5;
//        scanStartInd[i+1] = iter + 5;
    }

    lidar_raw_size = scan_ptr->size() ;

//    scanStartInd[0] = 5;
//    SelectPoints();

    ScanVoxelFilterDynamic();
    lidar_filter_size = filter_scan_ptr->size();
}

void SCANTOMAP::SelectPoints()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScanDS(new pcl::PointCloud<pcl::PointXYZI>());

    //    float cloudCurvature[PACKETNUM64 * 6 * 64 * 2];
    //    int cloudSortInd[PACKETNUM64 * 6 * 64 * 2];
    //    int cloudNeighborPicked[PACKETNUM64 * 6 * 64 * 2];
    //    int cloudLabel[PACKETNUM64 * 6 * 64 * 2];

    cornerPointsSharp->clear();
    cornerPointsLessSharp->clear();
    surfPointsFlat->clear();
    surfPointsLessFlat->clear();

    /// step1: calculate cloudCurvature, scanStartInd and scanEndInd
    int cloudSize=laserCloud->points.size();
    //    std::cout<<"cloudSize: "<<cloudSize<<std::endl;
    for(int i=5;i<cloudSize-5;i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;

        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
        //        std::cout<<"idx: "<<i<<std::endl;
    }

    //    std::cout<<"finish step1: "<<cloudSize<<std::endl;

    /// step2: calculate cloudNeighborPicked
    for (int i = 5; i < cloudSize - 6; i++) {
        float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
        float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
        float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
        float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

        if (diff > 0.1) { //0.1: the distance between two neighboring points should be larger than sqrt(0.1) = 0.3162 m

            float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                                laserCloud->points[i].y * laserCloud->points[i].y +
                                laserCloud->points[i].z * laserCloud->points[i].z);

            float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                    laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                    laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
            //the longer range point and his neighbour are possible occluded and impossible to be special points
            if (depth1 > depth2) {
                diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
                diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
                diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

                double dis = sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2;  // the normalized distance? it is equal to norm( points[i+1]/depth2 - points[i]/depth1 ).
                // Each of the two points are projected to the unit circle. theta = arccos((1+1-0.1^2)/2) = 5.72 degree
                // As long as the angle between these two points are less than 5.72degree, the following constraint would be satisfied
                if (dis < 0.1) {
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
            } else {
                diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
                diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
                diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

                if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
        }

        float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
        float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
        float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

        float dis = laserCloud->points[i].x * laserCloud->points[i].x
                + laserCloud->points[i].y * laserCloud->points[i].y
                + laserCloud->points[i].z * laserCloud->points[i].z;

        if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
            cloudNeighborPicked[i] = 1;
        }
    }

    //    std::cout<<" finish step2: "<<cloudSize<<std::endl;
    int maximum_beam_number = laserCloud->points[cloudSize-1].intensity;

    //    std::cout<<" maximum_beam_number: "<<maximum_beam_number<<std::endl;
    //maximum_beam_number=31;

    /// step3: compute feature points
    for(int i = 0; i < maximum_beam_number; i++) {
        surfPointsLessFlatScan->clear();
        for (int j = 0; j < 10; j++) {  // equal division into 10 parts for all points in each beam
            int sp = (scanStartInd[i] * (10 - j)  + scanEndInd[i] * j)/10;
            int ep = (scanStartInd[i] * (9 - j)  + scanEndInd[i] * (j + 1))/10-1;

            //arrange the curvature(smoothness) in ascending order for each part
            for (int k = sp + 1; k <= ep; k++) {
                for (int l = k; l >= sp + 1; l--) {
                    if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
                        int temp = cloudSortInd[l - 1];
                        cloudSortInd[l - 1] = cloudSortInd[l]; //exchange
                        cloudSortInd[l] = temp;
                    }
                }
            }

            //the large curvature is selected to be corner points and its small neighbour are impossible to be special points
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1) {  //0.1

                    largestPickedNum++;
                    if (largestPickedNum <= 3) { // 6 in CY   // 2 in FH
                        cloudLabel[ind] = 2;
                        cornerPointsSharp->push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                        //                        cout<<"@select corner"<<laserCloud->points[ind].x<<"  "<<laserCloud->points[ind].y<<"  "<<laserCloud->points[ind].z<<endl;
                    } else if (largestPickedNum <= 30) { // 70 in CY  // 20 in FH
                        cloudLabel[ind] = 1;  // corner points
                        cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                    } else {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        float diffX = laserCloud->points[ind + l].x
                                - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y
                                - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                                - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;// not select adjacent neighbour point
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = laserCloud->points[ind + l].x
                                - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y
                                - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                                - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            //the small curvature is selected to be surface points and its small neighbour are impossible to be special points
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++) {
                int ind = cloudSortInd[k];
                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1) {

                    cloudLabel[ind] = -1; //  surface
                    surfPointsFlat->push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {  // 8 in CY
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        float diffX = laserCloud->points[ind + l].x
                                - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y
                                - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                                - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        float diffX = laserCloud->points[ind + l].x
                                - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y
                                - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z
                                - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                if (cloudLabel[k] <= 0) {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        } // end single beam
        //        std::cout<<"end single beam"<<"idx: "<<i<<std::endl;
        surfPointsLessFlatScanDS->clear();
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);      // (0.3, 0.3, 0.3) in CY
        downSizeFilter.filter(*surfPointsLessFlatScanDS);

        *surfPointsLessFlat += *surfPointsLessFlatScanDS;
    } //end all beams

    mBuf.lock();
    surf_buf.push(surfPointsFlat);
    corn_buf.push(cornerPointsSharp);
    full_buf.push(scan_ptr);
    mBuf.unlock();
    pcl::io::savePCDFileBinary("cornerPointsSharp.pcd",*cornerPointsSharp);
    pcl::io::savePCDFileBinaryCompressed("surfPointsFlat.pcd",*surfPointsFlat);


}

void SCANTOMAP::GetCutVoxelMap(unordered_map<VOXEL_LOC, OCTO_TREE*> &feat_map, pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_feat,
                               Eigen::Matrix3d R_p, Eigen::Vector3d t_p, int feattype, int fnum, int capacity)
{
    uint plsize = pl_feat->size();
    for(uint i=0; i<plsize; i++)
    {
        // Transform point to world coordinate
        PointType &p_c = pl_feat->points[i];
        Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
        Eigen::Vector3d pvec_tran = R_p*pvec_orig + t_p;

        // Determine the key of hash table
        float loc_xyz[3];
        for(int j=0; j<3; j++)
        {
            loc_xyz[j] = pvec_tran[j] / voxel_size[feattype];
            if(loc_xyz[j] < 0)
            {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);

        // Find corresponding voxel
        auto iter = feat_map.find(position);
        if(iter != feat_map.end())
        {
            iter->second->plvec_orig[fnum]->push_back(pvec_orig);
            iter->second->plvec_tran[fnum]->push_back(pvec_tran);
            iter->second->is2opt = true;
        }
        else // If not finding, build a new voxel
        {
            OCTO_TREE *ot = new OCTO_TREE(feattype, capacity);
            ot->plvec_orig[fnum]->push_back(pvec_orig);
            ot->plvec_tran[fnum]->push_back(pvec_tran);

            // Voxel center coordinate
            ot->voxel_center[0] = (0.5+position.x) * voxel_size[feattype];
            ot->voxel_center[1] = (0.5+position.y) * voxel_size[feattype];
            ot->voxel_center[2] = (0.5+position.z) * voxel_size[feattype];
            ot->quater_length = voxel_size[feattype] / 4.0; // A quater of side length
            feat_map[position] = ot;
        }
    }
}

void SCANTOMAP::BAInitial()
{
    if(!bBA_init)
    {
        trans = Eigen::Matrix4d::Identity();
        kdtree_surf.reset((new pcl::KdTreeFLANN<PointType>()));
        kdtree_corn.reset((new pcl::KdTreeFLANN<PointType>()));
        pl_corn.reset(new pcl::PointCloud<PointType>);
        pl_surf.reset(new pcl::PointCloud<PointType>);
        // Number of received scans
        plcount = 0;
        // The sequence of frist scan in the sliding window.
        // Plus margi_size after every map-refine
        window_base = 0;
        accumulate_window = 1;
        surf_filter_length = 0.2;
        corn_filter_length = 0.0;
        window_size = 20;
        filter_num = 1;
        margi_size = 5;
        scan2map_on = 10;
        pub_skip = 5;
        printf("%lf %lf\n", voxel_size[0], voxel_size[1]);
        // LM optimizer for map-refine
        opt_lsv = new LM_SLWD_VOXEL(window_size, filter_num, thread_num);
        bBA_init = true;
    }
}

void SCANTOMAP::PointXYZI2PointType(pcl::PointCloud<pcl::PointXYZI> &pl, pcl::PointCloud<PointType> &plt)
{
    uint asize = pl.size();
    plt.resize(asize);

    for(uint i=0; i<asize; i++)
    {
        plt[i].x = pl[i].x;
        plt[i].y = pl[i].y;
        plt[i].z = pl[i].z;
        plt[i].intensity = pl[i].intensity;
    }
}

pcl::PointCloud<PointT>::Ptr SCANTOMAP::normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud)
{

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(10);
  ne.setViewPoint(0.0f, 0.0f, sensor_height);
  ne.compute(*normals);

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
  filtered->reserve(cloud->size());

  for(int i = 0; i < cloud->size(); i++) {
    float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
    if(std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
      filtered->push_back(cloud->at(i));
    }
  }

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

pcl::PointCloud<PointT>::Ptr SCANTOMAP::plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative)
{
  pcl::PlaneClipper3D<PointT> clipper(plane);
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

  clipper.clipPointCloud3D(*src_cloud, indices->indices);

  pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(src_cloud);
  extract.setIndices(indices);
  extract.setNegative(negative);
  extract.filter(*dst_cloud);

  return dst_cloud;
}

bool SCANTOMAP::PlaneDetection(const pcl::PointCloud<PointT>::Ptr& cloud, Eigen::Vector4f& plane)
{
      // compensate the tilt rotation
      Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();

      tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

      // filtering before RANSAC (height and normal filtering)
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
      pcl::transformPointCloud(*cloud, *filtered, tilt_matrix);
      filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);
      filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range), true);

      if(use_normal_filtering)
      {
        filtered = normal_filtering(filtered);
      }

      plane_lidar->clear();
      *plane_lidar =*filtered ;

      pcl::transformPointCloud(*filtered, *filtered, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

//      if(floor_filtered_pub.getNumSubscribers()) {
//        filtered->header = cloud->header;
//        floor_filtered_pub.publish(filtered);
//      }

//      std::cout << "filtered->size() : " << filtered->size() << std::endl;

      // too few points for RANSAC
      if(filtered->size() < floor_pts_thresh)
      {
//          std::cout << "filtered->size() : " << filtered->size() << std::endl;
        return false;
      }

      // RANSAC
      pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
      pcl::RandomSampleConsensus<PointT> ransac(model_p);
      ransac.setDistanceThreshold(0.1);
//      ransac.setOptimizeCoefficients(true);
//      ransac.setModelType (pcl::SACMODEL_PLANE);
//      ransac.setMethodType (pcl::SAC_RANSAC);
      ransac.computeModel();

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      ransac.getInliers(inliers->indices);

      // too few inliers
      if(inliers->indices.size() < floor_pts_thresh) {
        return false;
      }

      // verticality check of the detected floor's normal
      Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

      Eigen::VectorXf coeffs;
      ransac.getModelCoefficients(coeffs);

      double dot = coeffs.head<3>().dot(reference.head<3>());
      if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
        // the normal is not vertical
        return false;
      }

      // make the normal upward
      if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
        coeffs *= -1.0f;
      }

//      if(floor_points_pub.getNumSubscribers()) {
//        pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
//        pcl::ExtractIndices<PointT> extract;
//        extract.setInputCloud(filtered);
//        extract.setIndices(inliers);
//        extract.filter(*inlier_cloud);
//        inlier_cloud->header = cloud->header;

//        floor_points_pub.publish(inlier_cloud);
//      }

//      std::cout << "coeffs : " << coeffs << std::endl;
      plane = Eigen::Vector4f(coeffs);
      return true;

//      return Eigen::Vector4f(coeffs);
}



void SCANTOMAP::BundleAdustmentOdom()
{
    BAInitial();
    if(opt_lsv->read_refine_state()==2)
    {
        pl_send.clear();
        for(int i=0; i<margi_size; i+=pub_skip)
        {
            trans.block<3, 3>(0, 0) = opt_lsv->so3_poses[i].matrix();
            trans.block<3, 1>(0, 3) = opt_lsv->t_poses[i];
            pcl::PointCloud<PointType> pcloud;
            pcl::transformPointCloud(*pl_full_buf[window_base + i], pcloud, trans);

            pl_send += pcloud;
        }

        for(int i=0; i<margi_size; i++)
        {
            pl_full_buf[window_base + i] = nullptr;
        }

        for(int i=0; i<window_size; i++)
        {
            q_buf[window_base+i] = opt_lsv->so3_poses[i].unit_quaternion();
            t_buf[window_base+i] = opt_lsv->t_poses[i];
        }

        // Marginalization and update voxel map
        pl_surf_centor_map.clear(); pl_corn_centor_map.clear();
        for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
        {
            if(iter->second->is2opt)
            {
                iter->second->root_centors.clear();
                iter->second->marginalize(0, margi_size, q_buf, t_buf, window_base, iter->second->root_centors);
            }
            pl_surf_centor_map += iter->second->root_centors;
        }


        for(auto iter=corn_map.begin(); iter!=corn_map.end(); ++iter)
        {
            if(iter->second->is2opt)
            {
                iter->second->root_centors.clear();
                iter->second->marginalize(0, margi_size, q_buf, t_buf, window_base, iter->second->root_centors);
            }
            pl_corn_centor_map += iter->second->root_centors;
        }

        // window size of every voxel
        OCTO_TREE::voxel_windowsize -= margi_size;

        opt_lsv->free_voxel();
        window_base += margi_size; // as definition of window_base
        opt_lsv->set_refine_state(0);
    }

    if(surf_buf.empty() || corn_buf.empty() || full_buf.empty())
    {
        return;
    }

    mBuf.lock();
    pcl::PointCloud<PointType>::Ptr pl_full(new pcl::PointCloud<PointType>);

    PointXYZI2PointType(*surf_buf.front(), *pl_surf);
    PointXYZI2PointType(*corn_buf.front(), *pl_corn);
    PointXYZI2PointType(*full_buf.front(), *pl_full);

    corn_buf.pop(); surf_buf.pop(); full_buf.pop();

    std::cout << "pl_surf size : " << pl_surf->size() << std::endl;
    std::cout << "pl_corn size : " << pl_corn->size() << std::endl;

    if(pl_full->size()<5000)
    {
        mBuf.unlock();
        return;
    }

    pl_full_buf.push_back(pl_full);
    mBuf.unlock();

    plcount++;

    // The number of scans in the sliding window
    OCTO_TREE::voxel_windowsize = plcount - window_base;
    // Down sampling like PCL voxelgrid filter
    down_sampling_voxel(*pl_corn, corn_filter_length);

    // Scan2map module
    if(plcount > accumulate_window)
    {
        down_sampling_voxel(*pl_surf, surf_filter_length);

        // The new scan2map method needs several scans to initialize for Velodyne lidar
        if(plcount <= scan2map_on)
        {
            // Similar with loam mapping
            kdtree_surf->setInputCloud(pl_surf_fil_map.makeShared());
            kdtree_corn->setInputCloud(pl_corn_fil_map.makeShared());
        }
        else
        {
            // The new scan2map
            kdtree_surf->setInputCloud(pl_surf_centor_map.makeShared());
            kdtree_corn->setInputCloud(pl_corn_centor_map.makeShared());
        }

        // Two-step method
        for(int itercount=0; itercount<2; itercount++)
        {
            // LM optimizer for scan2map
            VOXEL_DISTANCE sld;
            sld.so3_pose.setQuaternion(q_curr);
            sld.t_pose = t_curr;

            a_size = pl_surf->size();
            if(plcount <= scan2map_on)
            {
                // The method is similar with loam mapping.
                for(uint i=0; i<a_size; i++)
                {
                    int ns = 5;
                    p_orig << (*pl_surf)[i].x, (*pl_surf)[i].y, (*pl_surf)[i].z;
                    aft_tran = q_curr*p_orig + t_curr;
                    apy.x = aft_tran[0]; apy.y = aft_tran[1]; apy.z = aft_tran[2];

                    kdtree_surf->nearestKSearch(apy, ns, pointSearchInd, pointSearchSqDis);

                    if(pointSearchSqDis[ns-1] > 5)
                    {
                        continue;
                    }

                    Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
                    Eigen::Vector3d center(0, 0, 0);
                    for(int j=0; j<ns; j++)
                    {
                        Eigen::Vector3d tvec;
                        tvec[0] = pl_surf_fil_map[pointSearchInd[j]].x;
                        tvec[1] = pl_surf_fil_map[pointSearchInd[j]].y;
                        tvec[2] = pl_surf_fil_map[pointSearchInd[j]].z;
                        covMat += tvec * tvec.transpose();
                        center += tvec;
                    }

                    center /= ns;
                    covMat -= ns * center * center.transpose();
                    covMat /= ns;

                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
                    if(saes.eigenvalues()[2] < 25*saes.eigenvalues()[0])
                    {
                        continue;
                    }

                    kervec = center;
                    orient = saes.eigenvectors().col(0);

                    range = fabs(orient.dot(aft_tran - kervec));

                    if(range > 1)
                    {
                        continue;
                    }

                    sld.push_surf(p_orig, kervec, orient, (1-0.75*range));
                }
            }
            else
            {
                // The scan2map method described in the paper.
                for(uint i=0; i<a_size; i++)
                {
                    int ns = 5;
                    p_orig << (*pl_surf)[i].x, (*pl_surf)[i].y, (*pl_surf)[i].z;
                    aft_tran = q_curr*p_orig + t_curr;
                    apy.x = aft_tran[0]; apy.y = aft_tran[1]; apy.z = aft_tran[2];

                    kdtree_surf->nearestKSearch(apy, ns, pointSearchInd, pointSearchSqDis);

                    if((pointSearchSqDis[0]) > 1.0)
                    {
                        continue;
                    }

                    // Find the nearest plane.
                    range = 10;
                    for(int j=0; j<ns; j++)
                    {
                        // The point in "pl_surf_centor_map" is defined below.
                        PointType &ay = pl_surf_centor_map[pointSearchInd[j]];
                        Eigen::Vector3d center(ay.x, ay.y, ay.z);
                        Eigen::Vector3d direct(ay.normal_x, ay.normal_y, ay.normal_z);
                        double dista = fabs(direct.dot(aft_tran - center));
                        if(dista <= range && pointSearchSqDis[j] < 4.0)
                        {
                            kervec = center;
                            orient = direct;
                            range = dista;
                        }
                    }

                    // Push points into optimizer
                    sld.push_surf(p_orig, kervec, orient, (1-0.75*range));
                }

                // Corn features
                a_size = pl_corn->size();
                for(uint i=0; i<a_size; i++)
                {
                    int ns = 3;
                    p_orig << (*pl_corn)[i].x, (*pl_corn)[i].y, (*pl_corn)[i].z;
                    aft_tran = q_curr*p_orig + t_curr;
                    apy.x = aft_tran[0]; apy.y = aft_tran[1]; apy.z = aft_tran[2];

                    kdtree_corn->nearestKSearch(apy, ns, pointSearchInd, pointSearchSqDis);
                    if((pointSearchSqDis[0]) > 1)
                    {
                        continue;
                    }

                    range = 10;
                    double dis_record = 10;
                    for(int j=0; j<ns; j++)
                    {
                        PointType &ay = pl_corn_centor_map[pointSearchInd[j]];
                        Eigen::Vector3d center(ay.x, ay.y, ay.z);
                        Eigen::Vector3d direct(ay.normal_x, ay.normal_y, ay.normal_z);
                        v_ac = aft_tran - center;
                        double dista = (v_ac - direct*direct.transpose()*v_ac).norm();
                        if(dista <= range)
                        {
                            kervec = center;
                            orient = direct;
                            range = dista;
                            dis_record = pointSearchSqDis[j];
                        }
                    }

                    if(range < 0.2 && sqrt(dis_record) < 1)
                    {
                        sld.push_line(p_orig, kervec, orient, (1-0.75*range));
                    }
                }
            }

            sld.damping_iter();
            q_curr = sld.so3_pose.unit_quaternion();
            t_curr = sld.t_pose;
        }
    }

    if(plcount <= scan2map_on)
    {
      trans.block<3, 3>(0, 0) = q_curr.matrix();
      trans.block<3, 1>(0, 3) = t_curr;

      pcl::transformPointCloud(*pl_surf, pl_send, trans);
      pl_surf_fil_map += pl_send;
      pcl::transformPointCloud(*pl_corn, pl_send, trans);
      pl_corn_fil_map += pl_send;
      down_sampling_voxel(pl_surf_fil_map, 0.2);
      down_sampling_voxel(pl_corn_fil_map, 0.2);
    }

    trans.block<3, 3>(0, 0) = q_curr.matrix();
    trans.block<3, 1>(0, 3) = t_curr;
    pcl::transformPointCloud(*pl_full, pl_send, trans);

    // Get the variation of pose
    if(plcount > 1)
    {
      delta_t = q_buf[plcount-2].matrix().transpose() * (t_curr-t_buf[plcount-2]);
      delta_q = q_buf[plcount-2].matrix().transpose() * q_curr.matrix();
    }

    q_buf.push_back(q_curr);
    t_buf.push_back(t_curr);
    delta_q_buf.push_back(delta_q);
    delta_t_buf.push_back(delta_t);

    if(plcount-window_base-window_size > 10)
    {
      printf("Out of size\n");
      return;
    }

    GetCutVoxelMap(surf_map, pl_surf, q_curr.matrix(), t_curr, 0, plcount-1-window_base, window_size+10);
    GetCutVoxelMap(corn_map, pl_corn, q_curr.matrix(), t_curr, 1, plcount-1-window_base, window_size+10);

    std::cout << "surf_map size : " << surf_map.size() << std::endl;
    std::cout << "corn_map size : " << corn_map.size() << std::endl;
    // The center point of surf points and corn points
    // The normal_x(yz) in each point is normal vector for plane
    // or direction vector for line.
    pl_surf_centor_map.clear(); pl_corn_centor_map.clear();

    // Points in new scan have been distributed in corresponding root node voxel
    // Then continue to cut the root voxel until right size
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
    {
      if(iter->second->is2opt) // Sliding window of root voxel should have points
      {
        iter->second->root_centors.clear();
        iter->second->recut(0, plcount-1-window_base, iter->second->root_centors);
      }

      // Add up surf centor points.
      pl_surf_centor_map += iter->second->root_centors;
      // You can add some distance restrictions in case that pl_surf_centor_map is too large.
      // You can also push points in root voxel into kdtree (loam mapping)
      // You can use "surf_map.erase(iter++)" to erase voxel for saving memory
    }

    for(auto iter=corn_map.begin(); iter!=corn_map.end(); ++iter)
    {
      if(iter->second->is2opt)
      {
        iter->second->root_centors.clear();
        iter->second->recut(0, plcount-1-window_base, iter->second->root_centors);
      }
      pl_corn_centor_map += iter->second->root_centors;
    }

    // Begin map refine module
    if(plcount>=window_base+window_size && opt_lsv->read_refine_state()==0)
    {
      for(int i=0; i<window_size; i++)
      {
        opt_lsv->so3_poses[i].setQuaternion(q_buf[window_base + i]);
        opt_lsv->t_poses[i] = t_buf[window_base + i];
      }

      // Do not optimize first sliding window
      if(window_base == 0)
      {
        opt_lsv->set_refine_state(2);
      }
      else
      {
        // Push voxel map into optimizer
        for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
        {
          if(iter->second->is2opt)
          {
            iter->second->traversal_opt(*opt_lsv);
          }
        }

        for(auto iter=corn_map.begin(); iter!=corn_map.end(); ++iter)
        {
          if(iter->second->is2opt)
          {
            iter->second->traversal_opt(*opt_lsv);
          }
        }

        // Begin iterative optimization
        // You can use multithreading or not.
        // We do not recommend use multiple thread on computer with poor performance

        // multithreading
        // map_refine_thread = new thread(&LM_SLWD_VOXEL::damping_iter, &opt_lsv);
        // map_refine_thread->detach();

        // non multithreading
        opt_lsv->damping_iter();
      }
    }

    // pose prediction
    t_curr = t_curr + q_curr * delta_t;
    q_curr = q_curr * delta_q;
}

void SCANTOMAP::update_fitness_score(const Eigen::Matrix4f& relpose)
{
    fitness_score_loop = calc_fitness_score(cureKeyframeCloud,prevKeyframeCloud,  relpose, match_score_range);
    fitness_score_loop = std::min(1000000.0, fitness_score_loop);
}

double SCANTOMAP::calc_fitness_score(const pcl::PointCloud<PointT>::Ptr& cloud1, const pcl::PointCloud<PointT>::Ptr& cloud2,
                                    const Eigen::Matrix4f& relpose, double max_range)
{
    pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());
    tree_->setInputCloud(cloud2);

    double fitness_score = 0.0;

    // Transform the input dataset using the final transformation
    pcl::PointCloud<PointT> input_transformed;
    pcl::transformPointCloud (*cloud1, input_transformed, relpose);

    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);

    // For each point in the source dataset
    int nr = 0;
    for (size_t i = 0; i < input_transformed.points.size (); ++i)
    {
        // Find its nearest neighbor in the target
        tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= max_range)
        {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (nr > 0)
        return (fitness_score / nr);
    else
        return (std::numeric_limits<double>::max ());
}



void  SCANTOMAP::processMatchingScore(pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr)
{
    if(_target_points->empty() || points_ptr == nullptr)
    {
        return;
    }

    matchingscore->SetInputTarget(map_ptr);
    updateMatchingScore(points_ptr);
}


void SCANTOMAP::getMatchAndUnmatchPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr match_points_ptr,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr unmatch_points_ptr)
{
    const auto point_with_distance_array = matchingscore->GetPointWithDistanceArray();
    for(const auto& point_with_distance : point_with_distance_array)
    {
        //more than score 50%
        if(point_with_distance.distance < matchingscore->GetFermiMu())
        {
            match_points_ptr->points.push_back(point_with_distance.point);
        }
        else
        {
            unmatch_points_ptr->points.push_back(point_with_distance.point);
        }
    }
}

void SCANTOMAP::updateMatchingScore(pcl::PointCloud<pcl::PointXYZI>::Ptr points_ptr)
{
    match_score = matchingscore->CalcMatchingScore(points_ptr);
}


//bool SCANTOMAP::SyncPackages()
//{
////    if(lidar_buffer_.empty() || imu_buffer_.empty())
////        return false;

////    if (last_timestamp_imu_ < lidar_end_time_) {
////        return false;
////    }
//}

void SCANTOMAP::Run()
{
    if(lidar_buffer_.empty())
        return;

    HDLADARDATA_MSG* msg_in = lidar_buffer_.front();
    lidar_buffer_.pop_front();
//    delete lidar_buffer_.front();
    Process(msg_in);
    keyNum = mMapDataFrame->cloud_keyposes_6d_ptr->size();


}

void SCANTOMAP::LidarCallBack(HDLADARDATA_MSG* msg)
{
    mutex_buffer_.lock();

//    if(msg->LocalPose.time <last_timestamp_lidar_)
//    {
//        lidar_buffer_.clear();
//    }

    last_timestamp_lidar_ =  msg->LocalPose.time;

    lidar_buffer_.push_back(msg);
    time_buffer_.push_back(last_timestamp_lidar_);
    mutex_buffer_.unlock();
}

void SCANTOMAP::IMUCallBack(LOCALPOSE_MSG* msg)
{
    double timestamp = msg->LocalPose.time;
    mutex_buffer_.lock();

    if(timestamp<last_timestamp_imu_)
       imu_buffer_.clear();
    last_timestamp_imu_ = msg->data.time;

    if(last_timestamp_imu_ <last_timestamp_lidar_)
    {
        imu_buffer_.pop_front();
    }

    imu_buffer_.emplace_back(msg);
    mutex_buffer_.unlock();
}
