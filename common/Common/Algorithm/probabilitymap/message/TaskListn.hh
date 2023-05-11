/*
TaskListn.hh

This C++ header file defines the NML Messages for TASKLIST
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef TASKLISTN_HH
#define TASKLISTN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.
#include "NMLmsgExn.hh"


// Define the integer type ids.
#define TASKLIST_MSG_TYPE 1103
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

struct GLOBAL_POINT
{
    double  x;                    // 坐标，localpose坐标系，单位cm
    double  y;

    char    turnInfo;             // 个位：路口，0:no turn, 1: left turn, 2: forward, 3: right turn, 4: u-turn,  5: 左前，6: 右前， 7: 左后，8: 右后，9: 转盘
                                  // 百位：0:FollowLane 1:FollowTask, 2:任意点导航

    unsigned short  pointType;    // 百位以内：
                                  // 0：默认；1：起点；2：终点；3：公交车停靠点；4：停车位出站入站点， 5: 停车位入站点，6: 停车位出站点，7: 物流车停靠点
                                  // 8：鸣笛点， 9：位于停靠范围内的点  10：巡逻点，11:充电点
                                  // 百位：表示改点需要精确到达
    char    reserve1;       // 0:默认； 1:珊格搜索结果
    short   angle;          // 角度信息,0-3600，Localose坐标系，单位0.1度. -1：不需要指定角度
    short   preferLaneID;   // 优先行驶车道编号，0：没有指定优先车道，1：默认右边第一条，2：默认右边第二条
    short   roadClass;      // 0:小区， 1: 城市， 2: 室内

    int     envType;        // 环境描述（0:非停止线，1: 停止线，即红绿灯等候点，2: 停车一段时间, 3：强制停车点）
    int     actType;        // 车辆行为描述
                            //个位：0：默认；1：起点；2：终点；3：公交车停靠点；4：停车位出站入站点， 5: 停车位入站点，6: 停车位出站点，7: 物流车停靠点
                    // 8：鸣笛点， 9：位于停靠范围内的点
                            //十位：0：默认非停靠区域；1：停靠区域；2：公交车站内区域， 3：通过门区域, 4:可倒车区域， 5：斜坡区域
                            //百位：0：默认前进点；1：后退点
    int     reserve2;       //
    int     maxSpeed;       // 限速,  km/h
    int     centerLineID;   // 矢量图中心线ID
    int     pointId;        // 全局id号
};


class TASKLIST_MSG : public NMLmsgEx
{
public:

    //Constructor
    TASKLIST_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    //PositionData Position;

    int pointNum;
    GLOBAL_POINT   globalPoints[500];

    int propNum;
    double         propArrays[500];

    GLOBAL_POINT   ReturnGlobalPoint;
    int cType;              //当前环境信息
    unsigned char taskState;       //定位错误标志位
    unsigned char locationSource;  //定位选择标志位
    int speedLimit;                //限速
    int prePointNum;
    GLOBAL_POINT   preGlobalPoints[50];
};

// Declare NML format function
extern int TaskListFormat(NMLTYPE, void *, CMS *);

#endif 	// TASKLISTN_HH

