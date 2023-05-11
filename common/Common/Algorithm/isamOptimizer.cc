/**
* Copyright(c)2017 ChangSha XingShen Technology Ltd .
* All rights reserved
* @projectName  CloudCompare-Qt
* @fileName     isamOptimizer.cc
* @brief
* @author       Yu Hui Liang
* @date         2020-08-14 18:09:48
*/
#include "isamOptimizer.hh"

ISAMOPTIMIZER::ISAMOPTIMIZER()
{
}

ISAMOPTIMIZER::~ISAMOPTIMIZER()
{
}

void ISAMOPTIMIZER::ResetIsam()
{
    isamParameters.relinearizeThreshold = 0.01;
    isamParameters.relinearizeSkip = 1;
//    gtsam::ISAM2DoglegParams dogleg_param;
//    dogleg_param.setVerbose(false);
//    isamParameters.setOptimizationParams(dogleg_param);
    this->isam_ptr.reset(new ISAM2(isamParameters));
    this->initial_estimate_ptr.reset(new Values);
    this->graph_ptr.reset(new NonlinearFactorGraph());
}

void ISAMOPTIMIZER::Update()
{
    this->isam_ptr->update(*this->graph_ptr,*this->initial_estimate_ptr);
    this->isam_ptr->update();
}

void ISAMOPTIMIZER::MutilUpdate()
{
    this->isam_ptr->update();
    this->isam_ptr->update();
    this->isam_ptr->update();
    this->isam_ptr->update();
    this->isam_ptr->update();
}

void ISAMOPTIMIZER::Reszie()
{
    this->graph_ptr->resize(0);
}

void ISAMOPTIMIZER::Clear()
{
    this->graph_ptr->resize(0);
    this->initial_estimate_ptr->clear();
}

gtsam::Values ISAMOPTIMIZER::CalculateEstimate()
{
    return this->isam_ptr->calculateEstimate();
}
