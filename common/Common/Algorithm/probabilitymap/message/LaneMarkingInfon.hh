/*
LaneMarkingInfon.hh

This C++ header file defines the NML Messages for LANEMARKINGINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LANEMARKINGINFON_HH
#define LANEMARKINGINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define LANEMARKINGINFO_MSG_TYPE 2103
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

#define MAX_LANEPT_NUM 200
// Define the NML Message Classes
struct LANE_OBJ
{
    INT32 LaneID;
    INT32 LaneRefer; // 0,1,-1,2,-2,-3,3, from left to right
    INT32 LaneAttr;  // 0: white real, 1:yellow real, 2: white xu, 3: yellow xu, 4: predicted
    INT32 LanePtNum;
    INT32 LanePtX[MAX_LANEPT_NUM];
    INT32 LanePtY[MAX_LANEPT_NUM];
    INT32 LaneProb; // 0-100, probability
    // line
    double a;
    double b;
    double c;
};

class LANEMARKINGINFO_MSG : public NMLmsgEx
{
public:

	//Constructor
	LANEMARKINGINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

    INT32 LaneNum;                             // lane num
    LANE_OBJ LaneObj[MAX_LANE_OBJ];
    INT32 CurrentLane;    // -100: on the left of all lanes, 100: on the right of all lanes
                          // -100<CurrentLane<100, vehicle is between lane CurrentLane and CurrentLane+1
};

// Declare NML format function
extern int LaneMarkingInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// LANEMARKINGINFON_HH
