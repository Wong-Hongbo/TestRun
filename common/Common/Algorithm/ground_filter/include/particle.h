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


#ifndef _PARTICLE_H_
#define _PARTICLE_H_
#include <iostream>
#include <vector>
#include <cmath>

// 物理常数
#define DAMPING 0.01f                       // 速度 阻尼系数
#define MAX_INF 9999999999
#define MIN_INF -9999999999
#define ACCELERATION -0.2f                  // 重力加速度

//we precompute the overall displacement of a particle accroding to the rigidness
// delta_z * 2 * k * (1 - k), RD=3, move_rate = 0.45
const double SingleMove[15] = { 0, 0.3, 0.51, 0.657, 0.7599, 0.83193, 0.88235, 0.91765,
                                 0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322 };
// delta_z * 2 * k * (1 - k), RD=3, move_rate = 0.49
const double DoubleMove[15] = { 0, 0.3, 0.42, 0.468, 0.4872, 0.4949, 0.498, 0.4992,
                                 0.4997, 0.4999, 0.4999, 0.5, 0.5, 0.5, 0.5 };

/* The particle class represents a particle that can move around in 3D space, mass==1*/
class Particle
{
private:
    bool occupy_flag_;                         // particle是否占据标志位
    bool movable_flag_;                        // particle是否可移动标志位
    bool visited_flag_;                        // particle是否被访问标志位
    float time_step2_;                         // particle内力位移时间步长
    float curr_z_;                             // particle的最新高度
    float last_z_;                             // particle的上一高度
    float reference_height_;                   // particle的地点参考高度
    float pos_x_;                              // particle的地点参考坐标x
    float pos_y_;                              // particle的地点参考坐标y
    std::vector<Particle *> neighbor_list_;    // particle的邻接列表
public:
    Particle(){ }
    void Init(float pos_z, float time_step2);
    inline void AddNeighbor(Particle *neighbor) { neighbor_list_.push_back(neighbor); }
    /* particle 直接力导致位置调整函数
     */
    void DirectForceActions();
    /* particle 间接力导致位置调整函数
     */
    void IndirectForceActions(int rigidness);
    inline bool GetOccupy() const { return occupy_flag_; }
    inline bool GetMovable() const { return movable_flag_; }
    inline bool GetVisited() const { return visited_flag_; }
    inline float GetPosX() const { return pos_x_; }
    inline float GetPosY() const { return pos_y_; }
    inline float GetCurrZ() const { return -curr_z_; }
    inline float GetGirdRefHeight() const { return reference_height_; }
    inline float GetMoveDis() const { return std::abs(curr_z_ - last_z_); }
    inline int GetNeighborNum() const { return neighbor_list_.size(); }
    inline Particle *GeiNeighborByIndex(int idx) const { return neighbor_list_[idx]; }
    inline void OffsetPos(const float v) { if (movable_flag_) curr_z_ += v; }
    inline void SetCurrZ(float z) { curr_z_ = z; }
    inline void SetPosition(float pos_x, float pos_y) { pos_x_ = pos_x; pos_y_ = pos_y;}
    inline void SetGirdRefHeight(float height) { reference_height_ = height; }
    inline void SetVisited(bool visited) { visited_flag_ = visited; }
    inline void SetUnmovable() { movable_flag_ = false; }
    inline void SetNeighborNum(int num) { neighbor_list_.reserve(num); }
    void SetOccupy(bool occupy_flag, float height);
    void VerifyBelowReferenceHeight();
};

#endif
