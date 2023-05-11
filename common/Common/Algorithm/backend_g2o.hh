/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     backend.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:06:18
*/
#ifndef BACK_END_HH_
#define BACK_END_HH_
#include "isamOptimizer.hh"
#include "Common/MapDataFrame/MapDataStruct.hh"
#include "Common/ParamServer.hh"
#include "Common/Commonfig.hh"
#include "mutex"

class ISAMOPTIMIZER;
class BACKEND
{
public:
    BACKEND(MAPDATAFRAME* MapDataFrame);
    ~BACKEND();

    void addOdomConstraint(const RPYpose current_pose, const RPYpose previous_pose, const int id);
    void addLoopConstraint(const int latest_id, const int closest_id,const RPYpose pose_from, const RPYpose pose_to);
    void addLoopConstConstraint(const int latest_id, const int closest_id, const RPYpose pose);
    void addGnssConstraint(const RPYpose gnss_pose);
    void addgroundPlaneConstraint();
    void removeOdomConstaint(const int start_id,const int end_id,const RPYpose pose_from, const RPYpose pose_to);
    void modifyOdomConstaint(const int start_id,const int end_id,const RPYpose pose_from, const RPYpose pose_to);
    void removeIndex();
    void SetNoise();
    void SetOptimateModel();
    void Clear();
    void Update();
    void MutilUpdate();
    void EstimateSize();

    // 当有prior gps 或者 闭环边记入，才能使用
    void CorrectKeyPose();
    MAPDATAFRAME *GetMapDataFrame();
    RPYpose Estimate();
    Eigen::MatrixXd GetPoseCovariance();

    void Reset();

    int current_keyframe_size;
    std::mutex mMutexReset;
    int current_optimite_size;
    gtsam::Values isam_current_estimate;

    ISAMOPTIMIZER* IsamOptimizer;

protected:
    MAPDATAFRAME* mMapDataFrame;

};


#endif//BACK_END_HH_
