#ifndef __LOGISTICSTASK__
#define __LOGISTICSTASK__

#include "rcs.hh"
#include "NMLmsgExn.hh"


#define LOGISTICSTASK_MSG_TYPE 18322


struct  TaskPoint
{
    double x;   
    double y;
    char   turn_info;// 停止线和路口，0:no turn, 1: left turn, 2: forward, 3: right turn, 4: u-turn,  5:路口出口点同时表示路口停止线位置
    unsigned short  light_type;// 个位：左转灯类型（0：无，1：圆灯，2：箭头），十位：直行灯类型（0：无，1：圆灯，2：箭头），
                               // 百位：右转灯类型（0：无，1：箭头），千位：uturn灯类型（0：无，1：掉头）,
                               // 万位：灯的排列方式（0：无信息，1：横排，2：竖排）

    char   light_pos_exit;    // 是否有灯的位置信息，0：没有，1：有
    short  light_x;    // 灯与当前全局点的相对位置和高度，单位为cm
    short  light_y;
    short  light_height;

    int    env_type;    // 环境描述（停止线，斑马线，行人，交通灯类型）
    int    act_type;    // 车辆行为描述（停车及持续时间，超车，换道）
    int    curve_radius;    // 曲率
    int    max_speed;    // 限速
    int    slope;    // 坡度
    int    point_id;    // 全局id号
};


class LOGISTICSTASK_MSG : public NMLmsgEx
{
public:
    LOGISTICSTASK_MSG();

    void update(CMS *);

    int point_num;    //任务点个数

    TaskPoint task_point[500];    //全局任务点

    int road_num;    //路的条数
    int       task_road[500];    //全局路id序列
};

// Declare NML format function
extern int LogisticsTaskFormat(NMLTYPE, void *, CMS *);

#endif