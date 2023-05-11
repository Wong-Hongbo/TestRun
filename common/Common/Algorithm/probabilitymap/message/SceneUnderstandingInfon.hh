/*
SceneUnderstandingInfon.hh

This C++ header file defines the NML Messages for SCENEUNDERSTANDINGINFO
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:05 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SCENEUNDERSTANDINGINFON_HH
#define SCENEUNDERSTANDINGINFON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"
#include "RoadInfon.hh"

// Define the integer type ids.
#define SCENEUNDERSTANDINGINFO_MSG_TYPE 2303
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
#define MAX_BRANCH_NUMBER 6

// 路口其中一条路的结构体
struct BRANCH_ROAD
{
    // 道路方向
    INT32 Direction;
    // 道路与路口的交叉点
    INT32 HaveLeftPt;
    INT32 LeftEdgePtX;
    INT32 LeftEdgePtY;
    INT32 HaveRightPt;
    INT32 RightEdgePtX;
    INT32 RightEdgePtY;
    INT32 HaveCenterPt;
    INT32 RoadCenterPtX;
    INT32 RoadCenterPtY;
};

struct INTERSECTION_OBJ
{
    INT32 BranchNum;
    // make sense when the BranchNum>1
    INT32 CurrentBranchRoad;
    BRANCH_ROAD  BranchRoad[MAX_BRANCH_NUMBER];
};

class SCENEUNDERSTANDINGINFO_MSG : public NMLmsgEx
{
public:

	//Constructor
	SCENEUNDERSTANDINGINFO_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 IntersectNum;
    INTERSECTION_OBJ IntersectObj[MAX_INTERSECTION_OBJ];

};

// Declare NML format function
extern int SceneUnderstandingInfoFormat(NMLTYPE, void *, CMS *);

#endif 	// SCENEUNDERSTANDINGINFON_HH
