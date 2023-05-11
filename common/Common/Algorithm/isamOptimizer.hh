/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     isamOptimizer.hh
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:06:24
*/
#ifndef ISAM_OPTIMIZER_HH_
#define ISAM_OPTIMIZER_HH_
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include "Common/ParamServer.hh"
using namespace gtsam;
class ISAMOPTIMIZER
{
public:
    ISAMOPTIMIZER();

    ~ISAMOPTIMIZER();

    boost::shared_ptr<NonlinearFactorGraph> graph_ptr;
    boost::shared_ptr<Values> initial_estimate_ptr;
    boost::shared_ptr<ISAM2> isam_ptr;

    ISAM2Params isamParameters;

    void ResetIsam();
    void Update();
    void Clear();
    void Reszie();
    void MutilUpdate();
    gtsam::Values CalculateEstimate();
    int EstimateSize();
    void SetNoise();
    void SetOptimateTpye();
    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

};


#endif//ISAM_OPTIMIZER_HH_
