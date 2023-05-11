/*
RemotePathn.hh

This C++ header file defines the NML Messages for REMOTEPATH
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef REMOTEPATHN_HH
#define REMOTEPATHN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"


// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define RemotePATHINFO_MSG_TYPE 189100
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
#define		MAXPLANPOINTS           50
#define     MAP_SIZE               10000
#ifndef     PI
#define     PI                     3.14159265358979323846
#endif

/*Remote coordinate*/
struct Remote_COORDINATE
{
    INT32		z_x;
    INT32		z_y;
    INT32		g_x;
    INT32		g_y;
    INT32		heading;		/*	UINT32		heading; */
    INT32		pitch;
    INT32		roll;
    INT32		height;
    UINT8		class_id;
    UINT8		reserved[3];
};

struct PATH_DATA
    {
    INT16 		x;
    INT16 		y;
};

struct PATH_DATA_PROPERTY
{
    PATH_DATA   left_boundary;
    PATH_DATA   right_boundary;
    INT32       main_direction;
};

enum SYS_STATE
{
        RS,RO,RT,RB,
        CN,CO,SG,
        HS,HF,HT,HB,
        RD,IT,MC,MR,EM,SP,FI
};

enum SYS_COMMAND
{
        ES,ST,AD,BK,IG,FO,PK
};

enum PLAN_STATE
{
    PP,NP,GT,LP,RR,WT,EC
};


struct RemotePLAN_PATH
{
    UINT32				time_flag;
    UINT32				plan_data_id; 		            	//局部路径规划帧号
    Remote_COORDINATE	plan_frame;				            //冻结坐标
    INT16				effective_point_num;	            //局部路径规划的有效点数目
    INT16				is_ok;					            //数据有效标志：0 - 无效; 1 - 有效
    PATH_DATA			plan_path[MAXPLANPOINTS];	       	//局部路径规划点集，有效点后数据补0
    PATH_DATA_PROPERTY	plan_path_property[MAXPLANPOINTS];
    SYS_COMMAND	        sys_command;		                //系统事件
    SYS_STATE			sys_state;				            //系统状态 
    PLAN_STATE			plan_state;				            //系统状态
    INT32				speed;                              //速度:公里/小时
    INT32				turning_index;						//转向状态: 1-直行；2-右转；3-左转；4-U-TURN；5-红灯停车
}  ;

class RemotePATHINFO_MSG : public NMLmsg
{
public:

	//Constructor
    RemotePATHINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    RemotePLAN_PATH path;
};

// Declare NML format function
extern int RemotePathInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// RemotePATHN_HH
