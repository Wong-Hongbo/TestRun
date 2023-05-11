/*
RoadInfon.hh

This C++ header file defines the NML Messages for ROADINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef ROADINFON_HH
#define ROADINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"

// Define the integer type ids.
#define ROADINFO_MSG_TYPE 2203
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

struct ROAD_OBJ
{
    INT32 RoadID;   //1 yellow
    INT32 LeftPtNum;
    INT32 RightPtNum;
    INT32 LeftEdgePtX[50];   //cm
    INT32 LeftEdgePtY[50];   //cm
    INT32 RightEdgePtX[50];  //cm
    INT32 RightEdgePtY[50];  //cm
    // format
};

struct YELLOW_LANE
{
    INT32 PtNum;    //
    INT32 LanePtX[50];   //cm
    INT32 LanePtY[50];   //cm
};

// Define the NML Message Classes

class ROADINFO_MSG : public NMLmsgEx
{
public:

	//Constructor
	ROADINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 MessageID;
 //   INT32 MessageSeqNum;
    INT32 TimeFlag;

    int RoadNum;                             // road num
    ROAD_OBJ RoadObj[MAX_ROAD_OBJ];

    YELLOW_LANE  YellowLane;
    INT32 YellowExist;   // 0, no yellow lane, >0, yellow exist
};

// Declare NML format function
extern int RoadInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// ROADINFON_HH
