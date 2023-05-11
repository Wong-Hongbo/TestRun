/*
RemoteCmdn.hh

This C++ header file defines the NML Messages for REMOTECMD
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef REMOTECMDN_HH
#define REMOTECMDN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define REMOTECMD_MSG_TYPE 152000
#define REMOTE_ACTUATOR_MSG_TYPE 152001
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes
//typedef enum{
//    EXECUTOR,SERVO,LOCAL_AIM_TRACKING,LOCAL_AIM_PLANNING,LOCAL_AIM_SENSING,GLOBAL_ROUTE,MISSION
//}REMOTE_MODE;


class REMOTE_CMD_DATA
{
public:
    // Place custom variables here.
    int MessageID;                      // 消息ID
    int TimeFlag;                       // 时间，单位ms
    int DestinationID;		            // 地址码，指定接收设备
    int Ugv_mode;	                    // 第一层控制命令，包括不同导航模式（自主，半自主，主从）以及传送全局任务 ，参照  UGV_MODE_CMD
    int Ugv_action;	                    // 第二层控制命令，不同导航模式下对应的动作 ，参照 UGV_ACTION_CMD
    int CommandData[4];                 // 控制数据，半自主和主从的时候，会发 MOTION_DATA
};

class REMOTECMD_MSG : public NMLmsg
{
public:

	//Constructor
	REMOTECMD_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    REMOTE_CMD_DATA data;
};
class REMOTE_ACTUATOR_MSG : public NMLmsg
{
public:

    //Constructor
    REMOTE_ACTUATOR_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    float   desired_steer;
    float   desired_brake;
    float   desired_trans;
    float   desired_fuel;
    int     remote_state;
    int     Vehicle_on;
    int      Starter_on;
};

// Declare NML format function
extern int RemoteCmdFormat(NMLTYPE, void *, CMS *);

#endif 	// REMOTECMDN_HH
