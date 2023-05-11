/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     backend.cc
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:10:02
*/
#include "backend.hh"

BACKEND::BACKEND(MAPDATAFRAME* MapDataFrame)
     :mMapDataFrame(MapDataFrame)
{
    IsamOptimizer = new ISAMOPTIMIZER();
    human_correct_pose = false;
//    VertexPtr = new g2o::VertexSE3();
}

BACKEND::~BACKEND()
{

}

void BACKEND::init()
{
    IsamOptimizer->ResetIsam();
}


void BACKEND::addVertx(const int id, const RPYpose current_pose)
{
    IsamOptimizer->initial_estimate_ptr->insert(gtsam::symbol('x',id),
                                                Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch, current_pose.yaw),
                                                      Point3(current_pose.x, current_pose.y, current_pose.z)));
}

void BACKEND::addEdge(const int start_id,const int end_id,const RPYpose pose, double nosie[6])
{
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) <<  1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

    gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(pose.roll, pose.pitch, pose.yaw), Point3(pose.x, pose.y, pose.z));
    IsamOptimizer->graph_ptr->add(BetweenFactor<Pose3>(gtsam::symbol('x',start_id),
                                                       gtsam::symbol('x',end_id), pose_from, odometryNoise));
}

void BACKEND::addPrior(const int id , const RPYpose current_pose,double nosie[6])
{
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());

    IsamOptimizer->graph_ptr->add(PriorFactor<Pose3>(gtsam::symbol('x',id), Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch, current_pose.yaw),
                                                                                  Point3(current_pose.x, current_pose.y, current_pose.z)), priorNoise));
}

//删除功能
void BACKEND::removeOdomConstaint(const int start_id,const int end_id,const RPYpose pose_from, const RPYpose pose_to)
{
//    RPYpose current_pose;
//    current_pose.x = mMapDataFrame->cloud_keyposes_6d_ptr->points[start_id].x;
//    current_pose.x = mMapDataFrame->cloud_keyposes_6d_ptr->points[start_id].y;
//    current_pose.x = mMapDataFrame->cloud_keyposes_6d_ptr->points[start_id].z;
//    current_pose.x = mMapDataFrame->cloud_keyposes_6d_ptr->points[start_id].roll;
//    current_pose.x = mMapDataFrame->cloud_keyposes_6d_ptr->points[start_id].pitch;
//    current_pose.x = mMapDataFrame->cloud_keyposes_6d_ptr->points[start_id].yaw;
////    IsamOptimizer->initial_estimate_ptr->erase(gtsam::symbol('x',start_id));
////    IsamOptimizer->initial_estimate_ptr->erase(gtsam::symbol('x',end_id));
////    IsamOptimizer->graph_ptr->remove(BetweenFactor<Pose3>(gtsam::symbol('x',start_id), gtsam::symbol('x',end_id),pose_from_in.between(pose_to_in) , odometryNoise));
//    IsamOptimizer->graph_ptr->erase(IsamOptimizer->graph_ptr->begin()+start_id,IsamOptimizer->graph_ptr->begin()+end_id);

    IsamOptimizer->graph_ptr->remove(start_id);
    IsamOptimizer->graph_ptr->remove(end_id);
}

// 修改功能 替换新的边
void BACKEND::modifyOdomConstaint(const int start_id,const int end_id,const RPYpose pose_from, const RPYpose pose_to)
{
////    IsamOptimizer->graph_ptr->remove(start_id);
////    IsamOptimizer->graph_ptr->remove(end_id);
////    IsamOptimizer->graph_ptr->replace(start_id,);
//    IsamOptimizer->graph_ptr->remove(start_id);
//    IsamOptimizer->graph_ptr->remove(end_id);

////    IsamOptimizer->initial_estimate_ptr->erase(gtsam::symbol('x',start_id));
////    IsamOptimizer->initial_estimate_ptr->erase(gtsam::symbol('x',end_id));

//    gtsam::Pose3 pose_from_in = Pose3(Rot3::RzRyRx(pose_from.roll, pose_from.pitch, pose_from.yaw), Point3(pose_from.x, pose_from.y, pose_from.z));
//    gtsam::Pose3 pose_to_in = Pose3(Rot3::RzRyRx(pose_to.roll, pose_to.pitch , pose_to.yaw), Point3(pose_to.x, pose_to.y, pose_to.z));

//    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
//    IsamOptimizer->graph_ptr->add(BetweenFactor<Pose3>(gtsam::symbol('x',start_id), gtsam::symbol('x',end_id),pose_from_in.between(pose_to_in) , odometryNoise));
}

void BACKEND::addLoopConstraint(const int latest_id, const int closest_id,const RPYpose pose_from, const RPYpose pose_to)
{
//    IsamOptimizer->ResetIsam();
//    std::cout << "latest_id : " << latest_id << std::endl;
    gtsam::Pose3 pose_from_in = Pose3(Rot3::RzRyRx(pose_from.roll, pose_from.pitch, pose_from.yaw), Point3(pose_from.x, pose_from.y, pose_from.z));
    gtsam::Pose3 pose_to_in = Pose3(Rot3::RzRyRx(pose_to.roll, pose_to.pitch , pose_to.yaw), Point3(pose_to.x, pose_to.y, pose_to.z));

//    std::cout << "closest_id1 : " << closest_id << std::endl;
    double scale_position = 0.00001;
    double scale_rotation = 0.00001;

    noiseModel::Diagonal::shared_ptr constraint_noise_;
    gtsam::Vector vector6(6);
    vector6 << scale_position, scale_position, scale_position, scale_rotation, scale_rotation, scale_rotation;
//    vector6 << 0.001, 0.001, 0.001, 0.15, 0.15, 0.15;
//        vector6 << 1e-6, 1e-6, 1e-6, 0.5, 0.5, 1e-4;
    constraint_noise_ = noiseModel::Diagonal::Variances(vector6);
//    std::cout << "closest_id2 : " << closest_id << std::endl;
    IsamOptimizer->graph_ptr->add(BetweenFactor<Pose3>(gtsam::symbol('x',latest_id), gtsam::symbol('x',closest_id), pose_from_in.between(pose_to_in), constraint_noise_));
//    std::cout << "closest_id3 : " << closest_id << std::endl;

//    auto loop_matrix = pose_from_in.between(pose_to_in);
//    Pointlooppose loop_pt;
//    loop_pt.loop_from = mMapDataFrame->latest_history_frame_id;
//    loop_pt.loop_from = mMapDataFrame->closest_history_frame_id;
//    loop_pt.x = loop_matrix.translation().x();
//    loop_pt.y = loop_matrix.translation().y();
//    loop_pt.z = loop_matrix.translation().z();
//    loop_pt.roll = loop_matrix.rotation().roll();
//    loop_pt.pitch = loop_matrix.rotation().pitch();
//    loop_pt.yaw = loop_matrix.rotation().yaw();

//    mMapDataFrame->cloud_loop_index->push_back(loop_pt);
}

void BACKEND::addLoopConstConstraint(const int latest_id, const int closest_id, const RPYpose pose)
{
    gtsam::Pose3 loop_pose = gtsam::Pose3(Rot3::RzRyRx(pose.roll, pose.pitch, pose.yaw),
                                          Point3(pose.x, pose.y,
                                                 pose.z));
//    double scale_position = 0.5;
//    double scale_rotation = 0.5;

    double scale_position = 0.00001;
    double scale_rotation = 0.00001;

    noiseModel::Diagonal::shared_ptr constraint_noise_;
    gtsam::Vector vector6(6);
    vector6 << scale_position, scale_position, scale_position, scale_rotation, scale_rotation, scale_rotation;


        constraint_noise_ = noiseModel::Diagonal::Variances(vector6);
//    gtsam::noiseModel::Robust::shared_ptr robust_loss =
//            gtsam::noiseModel::Robust::Create(
//                        gtsam::noiseModel::mEstimator::Cauchy::Create(1), constraint_noise_);

    IsamOptimizer->graph_ptr->add(BetweenFactor<Pose3>(gtsam::symbol('x',latest_id), gtsam::symbol('x',closest_id),loop_pose , constraint_noise_));

}


void BACKEND::addOdomConstraint(const RPYpose current_pose, const RPYpose previous_pose, const int id)
{
//    std::cout << " add odom id : " << id << std::endl;
    current_keyframe_size =id;
//    if(USE_BATCH_OPTIMIZER)
//    {
//        g2o::VertexSE3WithData* VertexNewPtr(new g2o::VertexSE3WithData);
//        double curr_estimate[7];
//        BatchOptimizer->RPYposeToMeasurement(current_pose, curr_estimate);
//        BatchOptimizer->CreateVertexSE3(curr_estimate,VertexNewPtr);

//        if(id==0)
//        {
//            VertexNewPtr->setFixed(true);
//            BatchOptimizer->AddVertex(VertexNewPtr);
//            VertexOldPtr = VertexNewPtr;
//            cout << "Set First Vertex!!" << endl;
//            return;
//        }
//        else
//        {

//        }
//    }
//    else
    {    if(id==0)
        {
            IsamOptimizer->ResetIsam();
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished()); // rad*rad, meter*meter

            IsamOptimizer->graph_ptr->add(PriorFactor<Pose3>(gtsam::symbol('x',0), Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch, current_pose.yaw),
                                                                                         Point3(current_pose.x, current_pose.y, current_pose.z)), priorNoise));
            IsamOptimizer->initial_estimate_ptr->insert(gtsam::symbol('x',0),
                                                        Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch, current_pose.yaw),
                                                              Point3(current_pose.x, current_pose.y, current_pose.z)));
        }
        else
        {
            gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(previous_pose.roll, previous_pose.pitch, previous_pose.yaw), Point3(previous_pose.x, previous_pose.y, previous_pose.z));
            gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch , current_pose.yaw), Point3(current_pose.x, current_pose.y, current_pose.z));

            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

            IsamOptimizer->graph_ptr->add(BetweenFactor<Pose3>(gtsam::symbol('x',id-1),
                                                               gtsam::symbol('x',id), pose_from.between(pose_to), odometryNoise));

//            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 0.5,0.5,0.5,0.5,0.5,0.5).finished()); // rad*rad, meter*meter

//            IsamOptimizer->graph_ptr->add(PriorFactor<Pose3>(gtsam::symbol('x',id), Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch, current_pose.yaw),
//                                                                                  Point3(current_pose.x, current_pose.y, current_pose.z)), priorNoise));


            IsamOptimizer->initial_estimate_ptr->insert(gtsam::symbol('x',id),
                                                        Pose3(Rot3::RzRyRx(current_pose.roll, current_pose.pitch, current_pose.yaw),
                                                              Point3(current_pose.x, current_pose.y, current_pose.z)));
            std::cout << "inc  "  << id-1 << " " << id << std::endl;
        }

    }

}


void BACKEND::addgroundPlaneConstraint()
{

}

void BACKEND::addGnssConstraint(const RPYpose gnss_pose)
{
    gtsam::Vector Vector3(3);
    Vector3 << 100, 100, 100;

    noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
    gtsam::GPSFactor gps_factor(gtsam::symbol('x',current_keyframe_size), gtsam::Point3(gnss_pose.x, gnss_pose.y,gnss_pose.z), gps_noise);
    IsamOptimizer->graph_ptr->add(gps_factor);
}

void BACKEND::Clear()
{
    IsamOptimizer->Clear();
}

void BACKEND::Resize()
{
    IsamOptimizer->Reszie();
}

void BACKEND::Update()
{
    IsamOptimizer->Update();
}

void BACKEND::MutilUpdate()
{
    IsamOptimizer->MutilUpdate();
}

RPYpose BACKEND::Estimate()
{
    isam_current_estimate = IsamOptimizer->CalculateEstimate();
    current_optimite_size = isam_current_estimate.size();
    std::cout << "current_optimate_size : " << current_optimite_size << std::endl;
    Pose3 latest_estimate = isam_current_estimate.at<Pose3>(gtsam::symbol('x',(current_optimite_size - 1)));

//    std::cout << "11111111111111111111111111111111111\n";
    RPYpose pose;
    pose.x =  latest_estimate.translation().x();
    pose.y =  latest_estimate.translation().y();
    pose.z =  latest_estimate.translation().z();
    pose.roll = latest_estimate.rotation().roll();
    pose.pitch = latest_estimate.rotation().pitch();
    pose.yaw  = latest_estimate.rotation().yaw();
//    std::cout << "pose.x :  " << pose.x << std::endl;
//    std::cout << "pose.y :  " << pose.y << std::endl;
//    std::cout << "pose.z :  " << pose.z << std::endl;
//    std::cout << "pose.roll :  " << pose.roll << std::endl;
//    std::cout << "pose.pitch :  " << pose.pitch << std::endl;
//    std::cout << "pose.yaw :  " << pose.yaw << std::endl;


    return pose;
}

Eigen::MatrixXd BACKEND::GetPoseCovariance()
{
   return IsamOptimizer->isam_ptr->marginalCovariance(gtsam::symbol('x',(current_optimite_size - 1)));
}

void BACKEND::Reset()
{
    IsamOptimizer->ResetIsam();
    mMapDataFrame->AllMapDataFrameMemory();
    mMapDataFrame->LoopClosingMapFrameMemory();
    mMapDataFrame->cloud_keyframes_v.clear();
}

void BACKEND::CorrectKeyPose()
{
    std::cout << "mMapDataFrame->cloud_keyposes_6d size : "<< mMapDataFrame->cloud_keyposes_6d_ptr->size() << std::endl;
    for (size_t i = 0; i < current_optimite_size; ++i)
    {
//        std::cout << "i : " << i << std::endl;
        mMapDataFrame->cloud_keyposes_6d_ptr->points[i].x = mMapDataFrame->cloud_keyposes_3d_ptr->points[i].x = mMapDataFrame->cloud_keyposes_2d_ptr->points[i].x
                = this->isam_current_estimate.at<Pose3>(gtsam::symbol('x',i)).translation().x();
        mMapDataFrame->cloud_keyposes_6d_ptr->points[i].y = mMapDataFrame->cloud_keyposes_3d_ptr->points[i].y = mMapDataFrame->cloud_keyposes_2d_ptr->points[i].y
                = this->isam_current_estimate.at<Pose3>(gtsam::symbol('x',i)).translation().y();
        mMapDataFrame->cloud_keyposes_6d_ptr->points[i].z = mMapDataFrame->cloud_keyposes_3d_ptr->points[i].z
                =  this->isam_current_estimate.at<Pose3>(gtsam::symbol('x',i)).translation().z();
        mMapDataFrame->cloud_keyposes_6d_ptr->points[i].roll = this->isam_current_estimate.at<Pose3>(gtsam::symbol('x',i)).rotation().roll();
        mMapDataFrame->cloud_keyposes_6d_ptr->points[i].pitch = this->isam_current_estimate.at<Pose3>(gtsam::symbol('x',i)).rotation().pitch();
        mMapDataFrame->cloud_keyposes_6d_ptr->points[i].yaw = this->isam_current_estimate.at<Pose3>(gtsam::symbol('x',i)).rotation().yaw();
    }
    pcl::io::savePCDFile("correct_keypose.pcd",*mMapDataFrame->cloud_keyposes_6d_ptr);
}



MAPDATAFRAME *BACKEND::GetMapDataFrame()
{
    return mMapDataFrame;
}



