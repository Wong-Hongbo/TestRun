/*
PlanningStatusn.hh
This C++ header file defines the NML Messages for PLANNINGSTATUS
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.


*/

// Prevent Multiple Inclusion
#ifndef PLANNINGSTATUSN_HH
#define PLANNINGSTATUSN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include    "GlobalPositionInfon.hh"
#include    "LocalPathInfon.hh"
#include    "BehaviorStatusn.hh"
#include "NMLmsgExn.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define PLANNINGSTATUS_MSG_TYPE 170600

#define POS_REFRESH                 1<<0
#define LPATH_REFRESH               1<<1
#define LOCAlPATHINFO_REFRESH       1<<2
#define LOCALDEMMAP_REFRESH         1<<4
#define PLANNINGSTATUS_BEHAVIORLIST_REFRESH        1<<6
#define PLANNINGSTATUS_GLOBALPOSITIONINFO_REFRESH  1<<7
#define PLANNINGSTATUS_LOCALATTRIBUTEMAP_REFRESH   1<<9
//#define

//typedef struct{
//    UINT32				time_flag;
//    UINT32				plan_data_id; 		            	/*	局部路径规划帧号			*/
//    LOCAL_COORDINATE	plan_frame;				            /*	冻结坐标        			*/
//    INT16				effective_point_num;	            /*	局部路径规划的有效点数目		*/
//    INT16				is_ok;					            /*	数据有效标志：0 - 无效; 1 - 有效	*/
//    WAYPOINT			plan_path[MAXPLANPOINTS];	       	/*	局部路径规划点集，有效点后数据补0 	*/
//    PATH_DATA_PROPERTY	plan_path_property[MAXPLANPOINTS];
//    VEHICLE_COMMAND     vehicle_command;		                /*	系统事件       */
//    SYS_STATE        	sys_state;
//    PLAN_STATE			plan_state;				            /*	系统状态        			*/
//    StateType           CurrentState;
//    INT32				speed;                              /*  速度:公里/小时                */
//    TURN_STATE			turning_state;
//    PATH_STATE          path_state;				/*	转向状态: 1-直行；2-右转；3-左转；4-U-TURN；5-红灯停车 */
//}Mnt_LOCALPLAN_PATH;
//typedef struct{
//    // Place custom variables here.
//    Mnt_LOCALPLAN_PATH LocalPathInfo_Data;
//}Mnt_LocalPathInfo;


//typedef struct{
//    // Place custom variables here.
//    INT32 MessageID;
//    INT32 MessageSeqNum;
//    INT32 TimeFlag;

//    PositionData Position;
//    INT32 MapWidth;
//    INT32 MapHeight;
//    INT32 GridWidth;
//    INT32 GridHeight;
//    INT32 VehicleGridX;
//    INT32 VehicleGridY;

//    UINT8 LocalDEMMap[LOCALDEM_MAP_SIZE];
//}Mnt_LocalDEMMap_Data;


// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class PLANNINGSTATUS_MSG : public NMLmsgEx
{
public:
	//Constructor
    PLANNINGSTATUS_MSG();
	// CMS Update Function
	void update(CMS *);

    int refreshFlag;
//    PLAN_STATE planState;

//    PositionData    Position;
//    LOCALPLAN_PATH       lpath;

//    Mnt_LocalPathInfo          localpathinfo; /* 9 */
//    Mnt_LocalDEMMap_Data       localDEMmap; /* 14 */
//    Mnt_BehaviorList           behaviorlist; /* 18 */
//    Mnt_GlobalPositionInfo     globalpositioninfo; /* 22 */
//    Mnt_LocalAttributeMap_Data localattributemap; /* 27 */

};




// Declare NML format function
extern int PlanningStatusFormat(NMLTYPE, void *, CMS *);

#endif 	// PLANNINGSTATUSN_HH
