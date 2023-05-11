/*
LocalPathInfon.hh

This C++ header file defines the NML Messages for LOCALPATHINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALPATHINFON_HH
#define LOCALPATHINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "OperatorCommandInfon.hh"
#include "BehaviorListn.hh"
#include "VehicleInfon.hh"
#include "NMLmsgExn.hh"
//ADD INDEX
//#include "EntityMapn.hh"
//#include "LocalDEMMapn.hh"
//#include "LocalAttributeMapn.hh"


// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

//////主要定义局部路径规划与外部输入输出接口的结构体，
//例如：融合后的地图数据结构体，惯性导航结构体，输出给控制的结构体，全局道路的结构体等
//
///惯性导航数据结构
typedef	struct {
    UINT32		time_flag;
    UINT8		frame_no;
    UINT8		error_no;
    INT32		z_x;    //cm
    INT32		z_y;
    INT32		g_x;
    INT32		g_y;
    UINT16		heading; //0.01deg
    INT16		pitch;
    INT16		roll;
    INT16		speed;  //cm/s
    INT16		height;
    UINT8		check;
}INERTIA_DATA_FGPL;


/////////////////下面为局部路径规划与行为决策之间的结构体//////


typedef enum {
    UNKNOWN=0,STRAIGHT_FORWARD,RIGHT_TURNING,LEFT_TURNING,U_TURN,
    STRAIGHT_BACKWARD,STOP2POINT,WAITING,PARKING
}VEHICLE_RUN_BEHAVIOR_0;



////////////////下面为局部路径规划，以及控制之间的结构体//////
typedef struct{
    INT16 		x;
    INT16 		y;
}WAYPOINT;


typedef struct {
    INT32 VehicleID;
    INT32 CenterX;
    INT32 CenterY;
    INT32  Width;
    INT32  Length;
    INT32   Heading;
    INT32 Speed;
    INT32 SpeedDirection;
    INT32 Height;          //cm
}VEHICLE2FOLLOW_0;


/* command to vehicle*/
typedef enum{
    ES=200,ST,AD_SPEED,AD_DISTANCE,AD_POINT,BK_SPEED,BK_POINT,IG,FO
}VEHICLE_COMMAND_0;
//ES:紧急停车；ST：停车；AD_SPEED:前进速度跟踪；AD_DISTANCE:前进定距跟踪；AD_POINT:前进定点停车；
//BK_SPEED:后退速度跟踪；BK_DISTANCE:后退dingju跟踪；IG：点火；FO：熄火。


typedef enum {
    RS,RO,RT,RB,
    CN,CO,SG,
    HS,HF,HT,HB,
    RD,IT,MC,MR,EM,SP,FI
} SYS_STATE_0;

//	RS-直道; RO-避障；RT-弯道；RB-分叉；
//	HS-高速直道；HF-高速尾随；HT-高速弯道；HB-高速分叉；
//	CN-越野；CO-越野避障；
//	MC-遥控；MR-遥控侦察；
//	IT-初始化；EM-异常；FI-终点停车 RD-准就绪 SP-停车
//  SG-Straight going

/* plan state */
typedef enum {
    PR,NP,GT,PB,RR,WT,EC,REPLAN,NORMAL_PLAN
}PLAN_STATE_0;
///////PR-得到规划路径    NP-没有道路可通行     EC-紧急情况
//////GT-到达目标点       PB－上一次的道路      RR-使用参考路
////// WT-waiting       REPLAN 重规划       NORMAL_PLAN


/*path data structure*/
typedef struct{
    WAYPOINT	left_boundary;
    WAYPOINT	right_boundary;
    INT32       direction;
}PATH_DATA_PROPERTY;


/////下面为局部路径规划和控制之间的通信的结构体//////////
typedef struct{
    UINT32				time_flag;
    UINT32				plan_data_id; 		            	/*	局部路径规划帧号			*/
    LOCAL_COORDINATE	plan_frame;				            /*	冻结坐标        			*/
    INT16				effective_point_num;	            /*	局部路径规划的有效点数目		*/
    INT16				is_ok;					            /*	数据有效标志：0 - 无效; 1 - 有效	*/
    WAYPOINT			path[MAXPLANPOINTS];	       	/*	局部路径规划点集，有效点后数据补0 	*/
    PATH_DATA_PROPERTY	path_property[MAXPLANPOINTS];
    VEHICLE_COMMAND_0     vehicle_command;		                /*	系统事件       */
    SYS_STATE_0        	sys_state;
    PLAN_STATE_0			plan_state;				            /*	系统状态        			*/
    INT32				speed;                                             /*  速度:公里/小时                */
    VEHICLE_RUN_BEHAVIOR_0  turning_state;
    StateType           CurrentState;
    VEHICLE2FOLLOW_0      Vehicle2Flo;
    BEHAVIOR_POINT      Point2Stop;
    float               expVelocity; //期望速度，值域0.0~100.0，单位是m/s
    float               expCurvature; //期望曲率，单位是1/m,满足右手规则
}LOCALPLAN_RESULT;

// Define the integer type ids.
#define LOCALPATHINFO_MSG_TYPE 189000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class LOCALPATHINFO_MSG : public NMLmsgEx
{
public:

    //Constructor
    LOCALPATHINFO_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    LOCALPLAN_RESULT LocalPathInfo;
    //ADD INDEX
    UINT32 index_behaviorlist;
    UINT32 index_entitymap;
    UINT32 index_localdemmap;
    UINT32 index_localattributemap;
};

// Declare NML format function
extern int LocalPathInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALPATHINFON_HH
