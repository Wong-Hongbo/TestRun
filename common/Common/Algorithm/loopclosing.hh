#ifndef LOOP_CLOSING_HH
#define LOOP_CLOSING_HH

#include "Common/MapDataFrame/MapDataStruct.hh"
#include "Common/GetMapType/MapTypeGenerate.hh"
#include "mutex"
#include <pcl/registration/icp.h>
#include "Common/Algorithm/match_core/ndt/ndt_omp.h"
#include "Common/Algorithm/backend.hh"
//#include "FrontEnd/front_end.hh"

class MAPDATAFRAME;
class MAPTYPEGENERATE;
class BACKEND;
class LOOPCLOSING
{
public:
    LOOPCLOSING(MAPDATAFRAME* MapDataFrame, FRONTEND* FrontEnd);
    ~LOOPCLOSING();

    void ProcessLoopClosing();
    bool DetectLoopClosure();
    void Run();

    void UpdateFitnessScore(const Eigen::Matrix4f& relpose);
    double CalcFitnessScore(const pcl::PointCloud<PointType3D>::Ptr& cloud1, const pcl::PointCloud<PointType3D>::Ptr& cloud2,
                                        const Eigen::Matrix4f& relpose, double max_range);
    MAPTYPEGENERATE *MapTypeGenerate;
//    BACKEND* BackEnd;
//    std::mutex mtx_;
    double diff_height;
    double height_last;
    double height_history;
    double fitness_score_loop;
    RPYpose loop_isam_pose;
    double shift_loop;

    void RequestFinish();
    void RequestStop();
    bool isFinished();
    bool isStopped();
    void Release();

private:

    bool CheckFinish();
    void SetFinish();
    std::mutex mMutexFinish;
    bool mbFinishRequested;
    bool Stop();
    std::mutex mMutexStop;
    bool mbStopped;
    bool mbStopRequested;
    bool mbFinished;

protected:
    MAPDATAFRAME* mMapDataFrame;
    FRONTEND* mFrontEnd;
};

#endif
