/*
BehaviorStatusn.hh
This C++ header file defines the NML Messages for BEHAVIORSTATUS
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.


*/

// Prevent Multiple Inclusion
#ifndef BEHAVIORSTATUSN_HH
#define BEHAVIORSTATUSN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "GlobalPositionInfon.hh"
#include "LocalPathInfon.hh"
#include "LaneMarkingInfon.hh"
#include "PedestrianInfon.hh"
#include "VehicleInfon.hh"
#include "RoadInfon.hh"
#include "TaskListn.hh"
#include "BehaviorListn.hh"
#include "NMLmsgExn.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define BEHAVIORSTATUS_MSG_TYPE 1205

/* Define the Data struct */
typedef struct{
    // Place custom variables here.
    // pedestrian vehicle road lanemark intersection
    int PdNum;
    PEDESTRIAN_OBJ PdObj[MAX_PEDESTRIAN_OBJ];

    int VehicleNum;
    VEHICLE_OBJ VehicleObj[MAX_VEHICLE_OBJ];

    int RoadNum;                             // road num
    ROAD_OBJ RoadObj[MAX_ROAD_OBJ];

    int LaneNum;
    LANE_OBJ LaneObj[MAX_LANE_OBJ];
}Mnt_EntityMap;

typedef struct{}GlobalMapInfo;

typedef struct{
    // Place custom variables here.
    // PositionData Position;
    INT32 pointNum;
    GLOBAL_POINT   globalPoints[2000];
}Mnt_TaskList;

typedef struct{}Mnt_BehaviorList;
typedef struct{}Mnt_GlobalMapInfo;

typedef struct{
    INT32 MessageID;
    INT32 MessageSeqNum;
    INT32 TimeFlag;

    // Place custom variables here.
    PositionData Position;
}Mnt_GlobalPositionInfo;

typedef struct{
    // Place custom variables here.
    INT32 MessageID;
    INT32 MessageSeqNum;
    INT32 TimeFlag;

    PositionData Position;

    INT32 MapWidth;
    INT32 MapHeight;
    INT32 GridWidth;
    INT32 GridHeight;
    INT32 VehicleGridX;
    INT32 VehicleGridY;

    UINT8 LocalAttributeMap[LOCAL_ATTRIBUTEMAP_SIZE];
}Mnt_LocalAttributeMap_Data;

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes

class BEHAVIORSTATUS_MSG : public NMLmsgEx
{
public:
	//Constructor
    BEHAVIORSTATUS_MSG();
	// CMS Update Function
	void update(CMS *);
	// Place custom variables here.

    int    refreshFlag;

    StateType CurrentState;

    PositionData           position; /* 0 */

    Mnt_EntityMap              entitymap; /* 9 */
    Mnt_GlobalMapInfo          globalmapinfo; /* 19 */
    Mnt_TaskList               tasklist; /* 21 */

    Mnt_BehaviorList           behaviorlist; /* 23 */
    Mnt_GlobalPositionInfo     globalpositioninfo; /* 25 */
    Mnt_LocalAttributeMap_Data localattributemap; /* 30 */
};



// Declare NML format function
extern int BehaviorStatusFormat(NMLTYPE, void *, CMS *);

#endif 	// BEHAVIORSTATUSN_HH
