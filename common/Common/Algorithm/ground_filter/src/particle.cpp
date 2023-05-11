//#######################################################################################
//#  This source code is about a ground filtering algorithm for airborn LiDAR data      #
//#  based on physical process simulations, specifically cloth simulation.              #
//#                                     Copyright 2021                                  #
//#                  Changsha Xingshen Intelligent Technology Co., Ltd                  #
//#                               (http://www.xingshentech.com/)                        #
//#                                                                                     #
//#                                  Authored By Meng Deyuan                            #
//#                                                                                     #
//#######################################################################################


#include "particle.h"


void Particle::Init(float pos_z, float time_step2)
{
    occupy_flag_ = false;
    movable_flag_ = true;
    visited_flag_ = false;
    time_step2_ = time_step2;
    curr_z_ = pos_z;
    last_z_ = pos_z;
    reference_height_ = MIN_INF;
}


void Particle::DirectForceActions()
{
    if (movable_flag_ && occupy_flag_)
    {
        float temp_z = curr_z_ + (curr_z_ - last_z_) * (1.0 - DAMPING) + ACCELERATION * time_step2_;
        last_z_ = curr_z_;
        curr_z_ = temp_z;
    }
}


// 内力移动，通过边来进行两次位移
void Particle::IndirectForceActions(int rigidness)
{
    Particle *p1 = this;
    if(p1->GetOccupy())
    {
        for (unsigned int i=0; i<neighbor_list_.size(); i++)
        {
            Particle *p2 = neighbor_list_[i];
            if(p2->GetOccupy())
            {
                float corr_z = p2->curr_z_ - p1->curr_z_;
                if (p1->GetMovable() && p2->GetMovable())
                {
                    corr_z = corr_z * DoubleMove[rigidness];
                    p1->OffsetPos(corr_z);
                    p2->OffsetPos(-corr_z);
                }
                else if (p1->GetMovable() && !p2->GetMovable())
                {
                    corr_z = corr_z * SingleMove[rigidness];
                    p1->OffsetPos(corr_z);
                }
                else if (!p1->GetMovable() && p2->GetMovable())
                {
                    corr_z = corr_z * SingleMove[rigidness];
                    p2->OffsetPos(-corr_z);
                }
            }
        }
    }
}


void Particle::SetOccupy(bool occupy_flag, float height)
{
    occupy_flag_ = occupy_flag;
    if(height > reference_height_)
    {
        reference_height_ = height;
    }
}


void Particle::VerifyBelowReferenceHeight()
{
    if(curr_z_ < reference_height_)
    {
        curr_z_ = reference_height_;
        last_z_ = reference_height_;
        movable_flag_ = false;
    }
}
