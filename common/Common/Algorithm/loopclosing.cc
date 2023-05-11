#include "loopclosing.hh"
#include "Common/ParamServer.hh"
LOOPCLOSING::LOOPCLOSING(MAPDATAFRAME* MapDataFrame, FRONTEND* FrontEnd):
    mMapDataFrame(MapDataFrame),mFrontEnd(FrontEnd),
    mbFinishRequested(false), mbFinished(false),mbStopRequested(false),mbStopped(false)
{

}

LOOPCLOSING::~LOOPCLOSING()
{

}

bool LOOPCLOSING::DetectLoopClosure()
{
    mMapDataFrame->latest_keyframe_ptr->clear();
    mMapDataFrame->near_history_keyframes_ptr->clear();
    std::lock_guard<std::mutex> lock(mMapDataFrame->mtx_);
    mMapDataFrame->latest_history_frame_id = mMapDataFrame->cloud_keyframes_v.size() - 1;
//    if(!accumulate_get_detect_loop_)
//    {
//        return false;
//    }

//    std::cout << "bMatchLoopId : " << bMatchLoopId << std::endl;

//    if(bMatchLoopId)
//    {
//        mMapDataFrame->closest_history_frame_id = loop_id;

//        RPYpose pose_trans;
//        pose_trans.x =     mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].x ;
//        pose_trans.y =     mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].y;
//        pose_trans.z =     mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].z;

//        pose_trans.roll =  mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].roll;
//        pose_trans.pitch = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].pitch;
//        pose_trans.yaw =   mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].yaw;

//        Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
//        RPYposeToMatrix(pose_trans,this_transformation);

//        pcl::transformPointCloud(mMapDataFrame->cloud_keyframes_v[mMapDataFrame->latest_history_frame_id], *mMapDataFrame->latest_keyframe_ptr, this_transformation);
//    }
//    else
    {
        double x_last = mMapDataFrame->cloud_keyposes_3d_ptr->points[mMapDataFrame->latest_history_frame_id].x ;
        double y_last = mMapDataFrame->cloud_keyposes_3d_ptr->points[mMapDataFrame->latest_history_frame_id].y ;
        pcl::PointXY cur_pose;
        cur_pose.x = x_last;
        cur_pose.y = y_last;

        mMapDataFrame->kdtree_pose_2d_ptr->setInputCloud(mMapDataFrame->cloud_keyposes_2d_ptr);

        mMapDataFrame->kdtree_pose_2d_ptr->radiusSearch(cur_pose, history_search_radius, search_idx_, search_dist_);
        //        kdtree_poses_->setInputCloud(cloud_keyposes_2d_);
        //        kdtree_poses_->radiusSearch(cur_pose, history_search_radius_, search_idx_, search_dist_);

        int current_id = mMapDataFrame->latest_history_frame_id;
        static int previous_id = current_id;
        double diff_id = current_id - previous_id ;

        //    bool close_failed=false;
        mMapDataFrame->closest_history_frame_id = -1;
//        std::cout << "latest_history_frame_id_ : " << latest_history_frame_id_ << std::endl;
        double min_dist=100000000.0;

        for (int i = 0; i < search_idx_.size(); ++i)
        {
            //        std::cout << "latest_history_frame_id_ - search_idx_[i] :  " << latest_history_frame_id_ - search_idx_[i] << std::endl;
            if (fabs(mMapDataFrame->latest_history_frame_id - search_idx_[i]) > index_detect)
            {
                double x_closed = mMapDataFrame->cloud_keyposes_3d_ptr->points[search_idx_[i]].x;
                double y_closed = mMapDataFrame->cloud_keyposes_3d_ptr->points[search_idx_[i]].y;
                double dist=sqrt((x_last-x_closed)*(x_last-x_closed)+(y_last-y_closed)*(y_last-y_closed));
                if(dist<min_dist)
                {
                    mMapDataFrame->closest_history_frame_id = search_idx_[i];
                    min_dist=dist;
                }
            }
        }

//        abs(cloud_keyposes_6d_->points[id].time - timeLaserOdometry) > 30.0;
//        std::cout << "time : " << cloud_keyposes_6d_->points[latest_history_frame_id_].time;

//        std::cout << "diff_id : " << diff_id << std::endl;
//        std::cout << "closest history frame id : " << closest_history_frame_id_ << std::endl;
        // 时间太短不做回环
        if (mMapDataFrame->closest_history_frame_id == -1 || diff_id==0)
        {
            return false;
        }

        previous_id = current_id;
//        task_status.status.task=1;
//        task_status.status.status=3;
//        strcpy(task_status.status.status_msg,"detect loop");

        //    *latest_keyframe_ +=*cloud_keyframes_[latest_history_frame_id_];
        height_last = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].z;
        height_history = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].z;
        diff_height = height_last - height_history;

        RPYpose pose_trans;
        pose_trans.roll = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].roll;
        pose_trans.pitch = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].pitch;
        pose_trans.yaw = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].yaw;
        pose_trans.x = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].x ;
        pose_trans.y =  mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].y ;
        pose_trans.z =  mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].z - diff_height;

        Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
        RPYposeToMatrix(pose_trans,this_transformation);

        std::cout << "this_transformation : " << this_transformation << std::endl;
        pcl::transformPointCloud(mMapDataFrame->cloud_keyframes_v[mMapDataFrame->latest_history_frame_id], *mMapDataFrame->latest_keyframe_ptr, this_transformation);
    }

//    SCclosestHistoryFrameID = -1; // init with -1
//    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
//    SCclosestHistoryFrameID = detectResult.first;
//    std::cout << "SC ID : " << SCclosestHistoryFrameID << std::endl;
//    yawDiffRad = detectResult.second; // not use /for v1 (because pcl icp withi initial somthing wrong...)


    //    pcl::copyPointCloud(*transformPointCloud(cloud_keyframes_[latest_history_frame_id_], cloud_keyposes_6d_->points[latest_history_frame_id_]), *latest_keyframe_);
//        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>());

//        for (int i = -history_search_num_, j; i <= history_search_num_; ++i)
//        {
//            j = closest_history_frame_id_ + i;
//            if (j < 0 || j >= latest_history_frame_id_)
//            {
//                continue;
//            }
//            *tmp_cloud += *transformPointCloud(cloud_keyframes_[j], cloud_keyposes_6d_->points[j]);
//        }
    *mMapDataFrame->near_history_keyframes_ptr += *MapTypeGenerate->GetTransformPoint3DCloud(mMapDataFrame->cloud_keyframes_v[mMapDataFrame->closest_history_frame_id].makeShared(),
            mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id]);

    //    }
    std::cout << "diff_height : "  << diff_height << std::endl;
//    LOG(INFO)<<"检测到闭环点，当前高程差为 = " << diff_height;
//        ds_history_keyframes_.setInputCloud(tmp_cloud);
//        ds_history_keyframes_.filter(*near_history_keyframes_);
    std::cout << "检测到闭环点..........................................................................需要闭环矫正\n";

//    b_batch_option = false;
    return true;
}


void LOOPCLOSING::Run()
{
    mbFinished = false;
    while(1)
    {
        ProcessLoopClosing();
        usleep(3000);
        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }
    SetFinish();

}

void LOOPCLOSING::ProcessLoopClosing()
{
    if (mMapDataFrame->cloud_keyposes_3d_ptr->points.empty())
    {
        return;
    }

    if (DetectLoopClosure() == false)
        return;

    auto start = std::chrono::system_clock::now();

    pcl::IterativeClosestPoint<PointType3D, PointType3D> icp;
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> registration_loop;
    Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f correction_frame;
    pcl::PointCloud<PointType3D>::Ptr unused_result(new pcl::PointCloud<PointType3D>());

    bool use_ndt_loop_match=false;
    bool has_converged=false;

    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(mMapDataFrame->latest_keyframe_ptr);
    icp.setInputTarget(mMapDataFrame->near_history_keyframes_ptr);

    initial_guess.block<3, 3>(0, 0) = (Eigen::AngleAxisf(mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].yaw, Eigen::Vector3f::UnitZ()) *
                                       Eigen::AngleAxisf(mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].pitch, Eigen::Vector3f::UnitY()) *
                                       Eigen::AngleAxisf(mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].roll, Eigen::Vector3f::UnitX()))
            .toRotationMatrix();

    initial_guess(0, 3) = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].x;
    initial_guess(1, 3) = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].y;
    initial_guess(2, 3) = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].z-diff_height;

    icp.align(*unused_result);
    has_converged = icp.hasConverged();
    correction_frame = icp.getFinalTransformation();

    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_keyframe_trans(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*mMapDataFrame->latest_keyframe_ptr,*latest_keyframe_trans,correction_frame);
//    processMatchingScore(latest_keyframe_trans,near_history_keyframes_);

    UpdateFitnessScore(correction_frame);

    std::cout << "fitness score single icp --------------------=" << fitness_score_loop << std::endl;

//    if(fitness_score_loop>history_fitness_score_ )
    if(1)
    {
        std::cout << "进入 ndt loop matching----------------------\n";
        use_ndt_loop_match =true;
        registration_loop.setTransformationEpsilon(0.000001);
        registration_loop.setResolution(2.0);

        if(env_UGVID=="c6")
        {
            registration_loop.setStepSize(0.1);
//            ;
        }
        else
        {
            registration_loop.setStepSize(0.5);
        }
        registration_loop.setNeighborhoodSearchMethod(pclomp::DIRECT1);
        registration_loop.setNumThreads(1);
        registration_loop.setMaximumIterations(64);
//        registration_loop.setInputTarget(near_history_keyframes_);
//        registration_loop.setInputSource(latest_keyframe_);
        registration_loop.setInputTarget(mMapDataFrame->latest_keyframe_ptr);
        registration_loop.setInputSource(mMapDataFrame->near_history_keyframes_ptr);

        initial_guess.block<3, 3>(0, 0) = (Eigen::AngleAxisf(mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].yaw, Eigen::Vector3f::UnitZ()) *
                                           Eigen::AngleAxisf(mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].pitch, Eigen::Vector3f::UnitY()) *
                                           Eigen::AngleAxisf(mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].roll, Eigen::Vector3f::UnitX()))
                .toRotationMatrix();

        initial_guess(0, 3) = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].x;
        initial_guess(1, 3) = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].y;
        initial_guess(2, 3) = mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->latest_history_frame_id].z-diff_height;

        registration_loop.align(*unused_result);
        has_converged = registration_loop.hasConverged();
//        fitness_score = registration_loop.getFitnessScore();
        correction_frame = registration_loop.getFinalTransformation().inverse();

        latest_keyframe_trans->clear();
        pcl::transformPointCloud(*mMapDataFrame->latest_keyframe_ptr,*latest_keyframe_trans,correction_frame);
//        processMatchingScore(latest_keyframe_trans,near_history_keyframes_);

        UpdateFitnessScore(correction_frame.inverse());
        std::cout << "fitness score single ndt ================  " << fitness_score_loop << std::endl;

//        if(readUGVID=="c6")
        {
            if(fitness_score_loop>history_fitness_score)
            {
//                registration_loop.setInputTarget(near_history_keyframes_);
//                registration_loop.setInputSource(latest_keyframe_trans);
                registration_loop.setInputTarget(mMapDataFrame->latest_keyframe_ptr);
                registration_loop.setInputSource(mMapDataFrame->near_history_keyframes_ptr);
//                registration_loop.setStepSize(0.2);
                registration_loop.align(*unused_result);
                correction_frame = registration_loop.getFinalTransformation().inverse();
                UpdateFitnessScore(correction_frame.inverse());
                std::cout << "fitness score single second ndt ================  " << fitness_score_loop << std::endl;
            }
        }

    }
    else
    {
        use_ndt_loop_match =false;
    }

//    if(bMatchLoopId)
//    {
//        //        bMatchLoopId =false;
//        // 在强制闭环时，将点云用历史帧的姿态进行变换，所以initial guess是历史帧的姿态
//        initial_guess.block<3, 3>(0, 0) = (Eigen::AngleAxisf(cloud_keyposes_6d_->points[closest_history_frame_id_].yaw, Eigen::Vector3f::UnitZ()) *
//                                           Eigen::AngleAxisf(cloud_keyposes_6d_->points[closest_history_frame_id_].pitch, Eigen::Vector3f::UnitY()) *
//                                           Eigen::AngleAxisf(cloud_keyposes_6d_->points[closest_history_frame_id_].roll, Eigen::Vector3f::UnitX()))
//                .toRotationMatrix();

//        initial_guess(0, 3) = cloud_keyposes_6d_->points[closest_history_frame_id_].x;
//        initial_guess(1, 3) = cloud_keyposes_6d_->points[closest_history_frame_id_].y;
//        initial_guess(2, 3) = cloud_keyposes_6d_->points[closest_history_frame_id_].z;
//    }

    Eigen::Quaternionf tmp_q(correction_frame.block<3, 3>(0, 0));
    double roll, pitch, yaw;
    XSTF::Matrix3x3(XSTF::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    Eigen::Matrix4f t_wrong = initial_guess;
    Eigen::Matrix4f t_correct = correction_frame * t_wrong;

    shift_loop = std::sqrt(std::pow(t_correct(0, 3) -   mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].x , 2)
                           + std::pow(t_correct(1, 3) - mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id].y, 2));
    std::cout << "fitness_score : " << fitness_score_loop << std::endl;
    std::cout << "has_converged : " << has_converged <<"   shift loop: "<< shift_loop<< std::endl;
    stringstream last_id;
    stringstream closet_id;

    last_id << save_loop_KeyFrame << "/" << "last_"<< mMapDataFrame->latest_history_frame_id << "_" << mMapDataFrame->closest_history_frame_id << "_" << fitness_score_loop << ".pcd";
    LOG(INFO) << "pcd last path  : " << last_id.str() ;
    closet_id << save_loop_KeyFrame << "/" <<"closest_" << mMapDataFrame->latest_history_frame_id << "_" <<mMapDataFrame->closest_history_frame_id<< "_" << fitness_score_loop << ".pcd";

    LOG(INFO) << "pcd clost path : " << closet_id.str() ;
    if(save_loop_result)
    {
        pcl::io::savePCDFileBinaryCompressed(last_id.str(),*latest_keyframe_trans);
        pcl::io::savePCDFileBinaryCompressed(closet_id.str(),*mMapDataFrame->near_history_keyframes_ptr);
    }
    LOG(INFO) << "history_fitness_score_ : " << history_fitness_score;

    if(use_ndt_loop_match)
    {
        if((has_converged == false ) || fitness_score_loop>history_fitness_score)
        {
            printf("ndt loop cannot closed\n");
//            task_status.status.task=1;
//            task_status.status.status=5;
//            strcpy(task_status.status.status_msg," loop cannot closed");
            printf("检测到ndt闭环点，但是没有满足约束条件:\n,"
                   "has_converged=%d, fitness_score_loop=%f,latest keyframe size=%d,闭环点id=%d， 当前点id=%d \n",has_converged,fitness_score_loop,mMapDataFrame->latest_keyframe_ptr->size(),mMapDataFrame->closest_history_frame_id,mMapDataFrame->latest_history_frame_id);
            return ;
        }
    }
    else
    {
//        if(use_gps_prior==1)
        {
            if ((has_converged == false ) || (fitness_score_loop > history_fitness_score) )
            {
                printf("loop cannot closed\n");
//                task_status.status.task=1;
//                task_status.status.status=5;
//                strcpy(task_status.status.status_msg," loop cannot closed");
                printf("检测到icp闭环点，但是没有满足约束条件:\n,"
                       "has_converged=%d, fitness_score=%f, shif_loop=%lf,latest keyframe size=%d,闭环点id=%d， 当前点id=%d\n",has_converged,fitness_score_loop,shift_loop,mMapDataFrame->latest_keyframe_ptr->size(),mMapDataFrame->closest_history_frame_id,mMapDataFrame->latest_history_frame_id);
                return;
            }
        }
//        else
//        {
//            if ( (has_converged == false ) || (fitness_score_loop > history_fitness_score_)  /*|| (match_score<0.6 )*/)
//            {
//                std::cout << "-------" << has_converged << " " << fitness_score_loop << " " << history_fitness_score_ << " " << 0 << std::endl;
//                printf("loop cannot closed\n");
//                task_status.status.task=1;
//                task_status.status.status=5;
//                strcpy(task_status.status.status_msg," loop cannot closed");
//                printf("检测到icp闭环点，但是没有满足约束条件:\n,"
//                       "has_converged=%d, fitness_score=%f, shif_loop=%lf,latest keyframe size=%d,闭环点id=%d， 当前点id=%d, match_score=%f\n",has_converged,fitness_score_loop,shift_loop,latest_keyframe_->size(),closest_history_frame_id_,latest_history_frame_id_,0);
//                return;
//            }
//        }
    }

    printf("开始拉！！！！闭环满足约束条件==================================>>>>>>\n "
           "fitness_score=%f,keyframe size=%d,闭环点id=%d， 当前点id=%d\n",fitness_score_loop, mMapDataFrame->latest_keyframe_ptr->size(), mMapDataFrame->closest_history_frame_id, mMapDataFrame->latest_history_frame_id);

    RPYpose pose_from,pose_to;
    MatrixToRPYpose(t_correct,pose_from);
    pose_to = PclToRPYpose(mMapDataFrame->cloud_keyposes_6d_ptr->points[mMapDataFrame->closest_history_frame_id]);

    std::lock_guard<std::mutex> lock(mMapDataFrame->mtx_);

//    Pose3 latest_estimate=isam_current_estimate_.at<Pose3>(gtsam::symbol('x',(this->isam_current_estimate_.size() - 1)));

    mFrontEnd->BackEnd->addLoopConstraint(mMapDataFrame->latest_history_frame_id,mMapDataFrame->closest_history_frame_id,pose_from,pose_to);
    mFrontEnd->BackEnd->Update();
    mFrontEnd->BackEnd->MutilUpdate();
    mFrontEnd->BackEnd->Clear();
    loop_isam_pose =mFrontEnd->BackEnd->Estimate();

    std::cout << "--------------------------loop result is ok -----------------------------" << std::endl;
    std::cout << "Time loop : " << elapsed.count() << std::endl;
    std::cout << "ICP has converged: " << has_converged << std::endl;
    std::cout << "Fitness score: " << fitness_score_loop << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    mMapDataFrame->loop_correct_closed = true;
}

void LOOPCLOSING::UpdateFitnessScore(const Eigen::Matrix4f& relpose)
{
    fitness_score_loop = CalcFitnessScore(mMapDataFrame->latest_keyframe_ptr,mMapDataFrame->near_history_keyframes_ptr, relpose, match_score_range);
    fitness_score_loop = std::min(1000000.0, fitness_score_loop);
}

double LOOPCLOSING::CalcFitnessScore(const pcl::PointCloud<PointType3D>::Ptr& cloud1, const pcl::PointCloud<PointType3D>::Ptr& cloud2,
                                    const Eigen::Matrix4f& relpose, double max_range)
{
    pcl::search::KdTree<PointType3D>::Ptr tree_(new pcl::search::KdTree<PointType3D>());
    tree_->setInputCloud(cloud1);

    double fitness_score = 0.0;

    // Transform the input dataset using the final transformation
    pcl::PointCloud<PointType3D> input_transformed;
    pcl::transformPointCloud (*cloud2, input_transformed, relpose);

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

void LOOPCLOSING::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LOOPCLOSING::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LOOPCLOSING::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LOOPCLOSING::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LOOPCLOSING::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool LOOPCLOSING::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LOOPCLOSING::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;
}

void LOOPCLOSING::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}
