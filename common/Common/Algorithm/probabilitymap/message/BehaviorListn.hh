/*
BehaviorPlanResultn.hh

This C++ header file defines the NML Messages for BEHAVIORPLANRESULT
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef BEHAVIORLISTN_HH
#define BEHAVIORLISTN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "VehicleInfon.hh"
#include "NMLmsgExn.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define BEHAVIORLIST_MSG_TYPE 1203
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

struct BEHAVIOR_POINT
{
    double x;
    double y;
};

enum StateType { WaitBegin  = 101,
                 WaitEnd    = 102,
                 WaitRoad   = 103,
                 WaitSide   = 104,
                 WaitParkBegin  = 105,
                 WaitParkEnd    = 106,

                 RunLane    = 201,
                 RunFollow  = 202,
                 RunPath    = 203,
                 RunLeft    = 204,
                 RunRight   = 205,
                 RunInter   = 206,
                 RunOvertake= 207,
                 RunChannel = 208,
                 RunLimit   = 209,

                 StartRoad  = 301,
                 StartSide  = 302,
                 StartPark  = 303,

                 ChangeLeft = 401,
                 ChangeRight= 402,

                 TurnLeft   = 501,
                 TurnRight  = 502,
                 TurnU      = 503,

                 StopPoint  = 601,
                 StopSide   = 602,

                 ParkV      = 701,
                 ParkSide   = 702,

                 HRunLane    = 801,
                 HRunFollow  = 802,
                 HRunPath    = 803,
                 HRunLeft    = 804,
                 HRunRight   = 805,
                 HRunInter   = 806,
                 HRunOvertake= 807,
                 HRunChannel = 808,
                 HRunBridge  = 809,
                 HRunRamp    = 810,
                 HRunStation = 811,
                 HMergeInto  = 812,
                 HGoRamp     = 813,

                 RePlan      = 901
               };

class BEHAVIORLIST_MSG : public NMLmsgEx
{
public:

	//Constructor
    BEHAVIORLIST_MSG();

	// CMS Update Function
	void update(CMS *);

    StateType CurrentState;
    VEHICLE_OBJ     Vehicle2Flo;
    VEHICLE_OBJ     Vehicle2Over;
    BEHAVIOR_POINT  Point2Stop;
    BEHAVIOR_POINT  Point2Park[4];
    double          MaxSpeed;
    double          MinSpeed;

 //   INT32 behaviorPointNum;
 //   BEHAVIOR_POINT   behaviorPoints[GLOBALPOINTNUM];

	// Place custom variables here.
};

// Declare NML format function
extern int BehaviorListFormat(NMLTYPE, void *, CMS *);

#endif 	// BEHAVIORLISTN_HH
